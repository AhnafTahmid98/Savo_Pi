"""Small helpers for keeping ROS topic names predictable."""

from __future__ import annotations


def normalize_topic(topic: str) -> str:
    value = str(topic).strip()

    if not value:
        raise ValueError("Topic name cannot be empty.")

    if not value.startswith("/"):
        value = f"/{value}"

    while "//" in value:
        value = value.replace("//", "/")

    if len(value) > 1 and value.endswith("/"):
        value = value.rstrip("/")

    return value


def join_topic(namespace: str, name: str) -> str:
    namespace = normalize_topic(namespace)
    name = str(name).strip().strip("/")

    if not name:
        raise ValueError("Topic suffix cannot be empty.")

    return normalize_topic(f"{namespace}/{name}")


def is_private_topic(topic: str) -> bool:
    return str(topic).strip().startswith("~")


def is_absolute_topic(topic: str) -> bool:
    return str(topic).strip().startswith("/")