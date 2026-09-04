from __future__ import annotations

import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time

PACKAGE = Path(__file__).resolve().parents[2]
SERVER = PACKAGE / 'scripts/fake_savomind_speech_server.py'
REQUEST_HEADER = struct.Struct('!8sIIIQ')
RESPONSE_HEADER = struct.Struct('!8sIIIIIIIIQ')
ACK_HEADER = struct.Struct('!8sIIIIII')
ACK_RESPONSE_HEADER = struct.Struct('!8sIIII')


def test_robot_side_runtime_is_connected_to_serialized_utterances() -> None:
    node = (PACKAGE / 'src/speech_node.cpp').read_text(encoding='utf-8')
    worker = (PACKAGE / 'src/transport/savomind_round_trip_worker.cpp').read_text(
        encoding='utf-8'
    )
    transport = (PACKAGE / 'src/transport/savomind_transport.cpp').read_text(
        encoding='utf-8'
    )
    assert 'initialize_savomind_runtime();' in node
    assert 'wait_serialized_for' in worker
    assert 'transport_.exchange(request)' in worker
    assert 'enqueue_playback' in worker
    assert 'wait_playback_completion_for' in worker
    assert 'transport_.acknowledge_playback(ack)' in worker
    assert 'SAVOMIND_SPEECH_PROTOCOL_VERSION = 2U' in (
        PACKAGE / 'include/savo_speech/transport/savomind_transport.hpp'
    ).read_text(encoding='utf-8')
    assert 'PLAYBACK_ACK_MAGIC' in transport
    assert '/savo_speech/transcript' in (
        PACKAGE / 'include/savo_speech/ros/topic_names.hpp'
    ).read_text(encoding='utf-8')


def receive_exact(client: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = client.recv(remaining)
        if not chunk:
            raise AssertionError('server closed before complete frame')
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def test_fake_server_implements_protocol_v2_and_playback_ack() -> None:
    # Darwin's sockaddr_un path limit is shorter than pytest's nested tmp_path.
    socket_root = Path('/private/tmp') if sys.platform == 'darwin' else Path(
        tempfile.gettempdir()
    )
    socket_path = socket_root / (
        f'savo-speech-{os.getpid()}-{time.monotonic_ns()}.sock'
    )
    process = subprocess.Popen(
        [sys.executable, str(SERVER), '--socket', str(socket_path), '--once'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        for _ in range(500):
            if socket_path.exists():
                break
            if process.poll() is not None:
                raise AssertionError(process.stderr.read())
            time.sleep(0.01)
        assert socket_path.exists(), 'fake speech server did not create its socket'
        request_id = b'request-1'
        session_id = b'session-1'
        wav = b'RIFF' + b'\0' * 40
        frame = REQUEST_HEADER.pack(
            b'SAVOSPRQ', 2, len(request_id), len(session_id), len(wav)
        ) + request_id + session_id + wav
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            client.sendall(frame)
            fields = RESPONSE_HEADER.unpack(
                receive_exact(client, RESPONSE_HEADER.size)
            )
            assert fields[0] == b'SAVOSPRS'
            assert fields[1] == 2
            assert fields[2] == 0
            assert fields[3] & 1
            body_size = sum(fields[4:9]) + fields[9]
            body = receive_exact(client, body_size)
            assert body[: fields[4]] == request_id
            token_offset = sum(fields[4:8])
            token = body[token_offset: token_offset + fields[8]]
            assert token == b'bench-playback-token'
            wav_offset = sum(fields[4:9])
            assert body[wav_offset: wav_offset + 4] == b'RIFF'
            assert len(body[wav_offset:]) == fields[9]

        reason = b''
        ack = ACK_HEADER.pack(
            b'SAVOSPAK',
            2,
            0,
            len(request_id),
            len(session_id),
            len(token),
            len(reason),
        ) + request_id + session_id + token + reason
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            client.sendall(ack)
            fields = ACK_RESPONSE_HEADER.unpack(
                receive_exact(client, ACK_RESPONSE_HEADER.size)
            )
            assert fields[0] == b'SAVOSPAR'
            assert fields[1] == 2
            assert fields[2] == 0
            body = receive_exact(client, fields[3] + fields[4])
            assert body[: fields[3]] == request_id
        assert process.wait(timeout=3) == 0
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3)
        socket_path.unlink(missing_ok=True)
