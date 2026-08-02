#!/usr/bin/env python3
"""
Deterministic bench-only server for Robot SAVO speech protocol v2.

This is not an STT, agent, or TTS implementation. It validates framing,
correlation, TTS playback, and the second-connection physical playback
acknowledgement required before pending navigation may dispatch.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import stat
import struct
import tempfile
import threading
import wave

SPEECH_REQUEST_MAGIC = b'SAVOSPRQ'
SPEECH_RESPONSE_MAGIC = b'SAVOSPRS'
PLAYBACK_ACK_MAGIC = b'SAVOSPAK'
PLAYBACK_ACK_RESPONSE_MAGIC = b'SAVOSPAR'
VERSION = 2
SPEECH_REQUEST_HEADER = struct.Struct('!8sIIIQ')
SPEECH_RESPONSE_HEADER = struct.Struct('!8sIIIIIIIIQ')
PLAYBACK_ACK_HEADER = struct.Struct('!8sIIIIII')
PLAYBACK_ACK_RESPONSE_HEADER = struct.Struct('!8sIIII')
MAX_ID = 128
MAX_TEXT = 8192
FLAG_PLAYBACK_ACK_REQUIRED = 1


def receive_exact(connection: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError('client disconnected')
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def make_wav(duration_ms: int = 180, sample_rate: int = 16000) -> bytes:
    with tempfile.SpooledTemporaryFile(max_size=1024 * 1024) as output:
        with wave.open(output, 'wb') as stream:
            stream.setnchannels(1)
            stream.setsampwidth(2)
            stream.setframerate(sample_rate)
            sample_count = sample_rate * duration_ms // 1000
            frames = bytearray()
            for index in range(sample_count):
                value = int(
                    1800
                    * math.sin(2.0 * math.pi * 440.0 * index / sample_rate)
                )
                frames.extend(struct.pack('<h', value))
            stream.writeframes(bytes(frames))
        output.seek(0)
        return output.read()


def encode_speech_response(
    request_id: bytes,
    wav_bytes: bytes,
    *,
    require_playback_ack: bool,
) -> bytes:
    transcript = b'robot savo bench speech request'
    reply = b'Robot SAVO speech transport is working.'
    reason = b'speech_pipeline_completed'
    token = b'bench-playback-token' if require_playback_ack else b''
    flags = FLAG_PLAYBACK_ACK_REQUIRED if require_playback_ack else 0
    header = SPEECH_RESPONSE_HEADER.pack(
        SPEECH_RESPONSE_MAGIC,
        VERSION,
        0,
        flags,
        len(request_id),
        len(reason),
        len(transcript),
        len(reply),
        len(token),
        len(wav_bytes),
    )
    return header + request_id + reason + transcript + reply + token + wav_bytes


def encode_ack_response(request_id: bytes, reason: bytes = b'event_accepted') -> bytes:
    return PLAYBACK_ACK_RESPONSE_HEADER.pack(
        PLAYBACK_ACK_RESPONSE_MAGIC,
        VERSION,
        0,
        len(request_id),
        len(reason),
    ) + request_id + reason


def handle_speech(
    connection: socket.socket,
    maximum_request_bytes: int,
    require_playback_ack: bool,
) -> str:
    rest = receive_exact(connection, SPEECH_REQUEST_HEADER.size - 8)
    version, request_id_size, session_id_size, wav_size = struct.unpack(
        '!IIIQ', rest
    )
    if version != VERSION:
        raise ValueError('protocol version mismatch')
    if request_id_size > MAX_ID or session_id_size > MAX_ID:
        raise ValueError('identity oversized')
    body_size = request_id_size + session_id_size + wav_size
    if wav_size < 44 or body_size > maximum_request_bytes:
        raise ValueError('request oversized')
    body = receive_exact(connection, body_size)
    request_id = body[:request_id_size]
    response = encode_speech_response(
        request_id,
        make_wav(),
        require_playback_ack=require_playback_ack,
    )
    connection.sendall(response)
    return 'speech'


def handle_playback_ack(
    connection: socket.socket,
    maximum_request_bytes: int,
) -> str:
    rest = receive_exact(connection, PLAYBACK_ACK_HEADER.size - 8)
    (
        version,
        playback_status,
        request_id_size,
        session_id_size,
        token_size,
        reason_size,
    ) = struct.unpack('!IIIIII', rest)
    if version != VERSION or playback_status not in {0, 1}:
        raise ValueError('invalid playback ack header')
    if request_id_size > MAX_ID or session_id_size > MAX_ID:
        raise ValueError('playback identity oversized')
    if token_size > MAX_TEXT or reason_size > MAX_TEXT:
        raise ValueError('playback text oversized')
    body_size = request_id_size + session_id_size + token_size + reason_size
    if body_size > maximum_request_bytes:
        raise ValueError('playback ack oversized')
    body = receive_exact(connection, body_size)
    request_id = body[:request_id_size]
    token_offset = request_id_size + session_id_size
    token = body[token_offset: token_offset + token_size]
    if token != b'bench-playback-token':
        raise ValueError('wrong playback token')
    connection.sendall(encode_ack_response(request_id))
    return 'playback_ack'


def handle_client(
    connection: socket.socket,
    maximum_request_bytes: int,
    require_playback_ack: bool,
) -> str:
    with connection:
        magic = receive_exact(connection, 8)
        if magic == SPEECH_REQUEST_MAGIC:
            return handle_speech(
                connection,
                maximum_request_bytes,
                require_playback_ack,
            )
        if magic == PLAYBACK_ACK_MAGIC:
            return handle_playback_ack(connection, maximum_request_bytes)
        raise ValueError('unknown request magic')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--socket',
        type=Path,
        default=Path('/tmp/savomind-speech-test.sock'),
    )
    parser.add_argument(
        '--maximum-request-bytes',
        type=int,
        default=2 * 1024 * 1024,
    )
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--no-playback-ack', action='store_true')
    args = parser.parse_args()
    if not args.socket.is_absolute():
        parser.error('--socket must be absolute')
    if args.maximum_request_bytes < 44:
        parser.error('--maximum-request-bytes must be at least 44')

    args.socket.parent.mkdir(parents=True, exist_ok=True)
    try:
        status = os.lstat(args.socket)
    except FileNotFoundError:
        pass
    else:
        if stat.S_ISLNK(status.st_mode) or not stat.S_ISSOCK(status.st_mode):
            parser.error('existing socket path is unsafe')
        args.socket.unlink()
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(args.socket))
    os.chmod(args.socket, 0o660)
    listener.listen(4)
    listener.settimeout(0.5)
    stopping = threading.Event()

    def stop_handler(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    print(f'fake SavoMind speech server v2 listening on {args.socket}', flush=True)
    speech_seen = False
    ack_seen = False
    require_ack = not args.no_playback_ack
    try:
        while not stopping.is_set():
            try:
                connection, _ = listener.accept()
            except TimeoutError:
                continue
            try:
                kind = handle_client(
                    connection,
                    args.maximum_request_bytes,
                    require_ack,
                )
                speech_seen = speech_seen or kind == 'speech'
                ack_seen = ack_seen or kind == 'playback_ack'
            except Exception as exc:
                print(f'fake server request failed: {exc}', flush=True)
            if args.once and speech_seen and (ack_seen or not require_ack):
                break
    finally:
        listener.close()
        try:
            args.socket.unlink()
        except FileNotFoundError:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
