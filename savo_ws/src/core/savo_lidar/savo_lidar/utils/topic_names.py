# -*- coding: utf-8 -*-
"""Topic-name helpers."""

from __future__ import annotations


def normalize_topic(topic: str) -> str:
    value = str(topic).strip()

    if not value:
        raise ValueError("topic cannot be empty")

    if is_private_topic(value):
        return value

    if not value.startswith("/"):
        value = f"/{value}"

    while "//" in value:
        value = value.replace("//", "/")

    if len(value) > 1:
        value = value.rstrip("/")

    return value


def is_absolute_topic(topic: str) -> bool:
    return str(topic).strip().startswith("/")


def is_private_topic(topic: str) -> bool:
    return str(topic).strip().startswith("~")


def join_topic(*parts: str) -> str:
    clean_parts = []

    for part in parts:
        value = str(part).strip().strip("/")

        if value:
            clean_parts.append(value)

    if not clean_parts:
        raise ValueError("topic parts cannot be empty")

    return normalize_topic("/".join(clean_parts))


__all__ = [
    "is_absolute_topic",
    "is_private_topic",
    "join_topic",
    "normalize_topic",
]
