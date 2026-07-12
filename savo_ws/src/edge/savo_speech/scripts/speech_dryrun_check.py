#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Exercise the complete Phase 0D dry-run ROS runtime."""

from __future__ import annotations

import json
import sys
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def heartbeat_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class SpeechDryrunChecker(Node):
    def __init__(self) -> None:
        super().__init__("speech_dryrun_check")

        self.state: str | None = None
        self.wake_state: str | None = None
        self.listening: bool | None = None
        self.input_muted: bool | None = None
        self.output_muted: bool | None = None
        self.status: dict | None = None
        self.heartbeat_count = 0

        self.create_subscription(
            String,
            "/savo_speech/state",
            self._on_state,
            state_qos(),
        )

        self.create_subscription(
            String,
            "/savo_speech/wake_state",
            self._on_wake_state,
            state_qos(),
        )

        self.create_subscription(
            Bool,
            "/savo_speech/listening",
            self._on_listening,
            state_qos(),
        )

        self.create_subscription(
            Bool,
            "/savo_speech/input_muted",
            self._on_input_muted,
            state_qos(),
        )

        self.create_subscription(
            Bool,
            "/savo_speech/output_muted",
            self._on_output_muted,
            state_qos(),
        )

        self.create_subscription(
            String,
            "/savo_speech/status",
            self._on_status,
            status_qos(),
        )

        self.create_subscription(
            String,
            "/savo_speech/heartbeat",
            self._on_heartbeat,
            heartbeat_qos(),
        )

        self.clients = {
            "wake": self.create_client(
                Trigger,
                "/savo_speech/wake",
            ),
            "sleep": self.create_client(
                Trigger,
                "/savo_speech/sleep",
            ),
            "start_listening": self.create_client(
                Trigger,
                "/savo_speech/start_listening",
            ),
            "stop_listening": self.create_client(
                Trigger,
                "/savo_speech/stop_listening",
            ),
            "cancel": self.create_client(
                Trigger,
                "/savo_speech/cancel",
            ),
            "mute_input": self.create_client(
                SetBool,
                "/savo_speech/mute_input",
            ),
            "mute_output": self.create_client(
                SetBool,
                "/savo_speech/mute_output",
            ),
            "reload_audio_device": self.create_client(
                Trigger,
                "/savo_speech/reload_audio_device",
            ),
        }

    def _on_state(self, message: String) -> None:
        self.state = message.data

    def _on_wake_state(self, message: String) -> None:
        self.wake_state = message.data

    def _on_listening(self, message: Bool) -> None:
        self.listening = message.data

    def _on_input_muted(self, message: Bool) -> None:
        self.input_muted = message.data

    def _on_output_muted(self, message: Bool) -> None:
        self.output_muted = message.data

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)

    def _on_heartbeat(self, message: String) -> None:
        json.loads(message.data)
        self.heartbeat_count += 1

    def wait_until(
        self,
        predicate: Callable[[], bool],
        description: str,
        timeout_s: float = 5.0,
    ) -> None:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            if predicate():
                return

        raise RuntimeError(
            f"Timed out waiting for: {description}"
        )

    def wait_for_services(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s

        for name, client in self.clients.items():
            remaining = deadline - time.monotonic()

            if remaining <= 0.0:
                raise RuntimeError(
                    f"Timed out waiting for service: {name}"
                )

            if not client.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(
                    f"Service unavailable: {name}"
                )

    def call_trigger(self, name: str):
        client = self.clients[name]
        future = client.call_async(Trigger.Request())

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                f"Trigger service failed: {name}"
            )

        response = future.result()

        if not response.success:
            raise RuntimeError(
                f"{name} rejected request: {response.message}"
            )

        return response

    def call_set_bool(self, name: str, value: bool):
        request = SetBool.Request()
        request.data = value

        client = self.clients[name]
        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                f"SetBool service failed: {name}"
            )

        response = future.result()

        if not response.success:
            raise RuntimeError(
                f"{name} rejected request: {response.message}"
            )

        return response

    def run_validation(self) -> None:
        print("[1/10] Waiting for services")
        self.wait_for_services()

        print("[2/10] Checking initial sleeping state")
        self.wait_until(
            lambda: (
                self.state == "SLEEPING"
                and self.wake_state == "SLEEPING"
                and self.listening is False
                and self.input_muted is False
                and self.output_muted is False
                and self.status is not None
                and self.heartbeat_count > 0
            ),
            "initial speech state",
        )

        print("[3/10] Calling wake")
        self.call_trigger("wake")
        self.wait_until(
            lambda: (
                self.state == "AWAKE_IDLE"
                and self.wake_state == "AWAKE"
            ),
            "awake idle state",
        )

        print("[4/10] Starting listening")
        self.call_trigger("start_listening")
        self.wait_until(
            lambda: (
                self.state == "LISTENING"
                and self.listening is True
            ),
            "listening state",
        )

        print("[5/10] Stopping listening")
        self.call_trigger("stop_listening")
        self.wait_until(
            lambda: (
                self.state == "AWAKE_IDLE"
                and self.listening is False
            ),
            "listening stopped",
        )

        print("[6/10] Testing microphone mute")
        self.call_set_bool("mute_input", True)
        self.wait_until(
            lambda: self.input_muted is True,
            "input muted",
        )

        self.call_set_bool("mute_input", False)
        self.wait_until(
            lambda: self.input_muted is False,
            "input unmuted",
        )

        print("[7/10] Testing speaker mute")
        self.call_set_bool("mute_output", True)
        self.wait_until(
            lambda: self.output_muted is True,
            "output muted",
        )

        self.call_set_bool("mute_output", False)
        self.wait_until(
            lambda: self.output_muted is False,
            "output unmuted",
        )

        print("[8/10] Reloading dry-run audio device")
        self.call_trigger("reload_audio_device")

        print("[9/10] Testing cancellation")
        self.call_trigger("cancel")
        self.wait_until(
            lambda: (
                self.state == "AWAKE_IDLE"
                and self.listening is False
            ),
            "cancelled awake idle state",
        )

        print("[10/10] Returning to sleep")
        self.call_trigger("sleep")
        self.wait_until(
            lambda: (
                self.state == "SLEEPING"
                and self.wake_state == "SLEEPING"
            ),
            "final sleeping state",
        )

        if self.status is None:
            raise RuntimeError("No status message received")

        required_status_keys = {
            "available",
            "ok",
            "ready_for_voice",
            "state",
            "wake_state",
            "listening",
            "tts_gate_active",
            "face_state",
            "last_error",
        }

        missing = required_status_keys - set(self.status)

        if missing:
            raise RuntimeError(
                f"Status is missing fields: {sorted(missing)}"
            )

        print()
        print("Phase 0D dry-run runtime validation: PASSED")


def main() -> int:
    rclpy.init()
    node = SpeechDryrunChecker()

    try:
        node.run_validation()
        return 0
    except Exception as exc:
        print(
            f"Phase 0D dry-run runtime validation: FAILED: {exc}",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
