#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Topic-name helpers for Robot Savo localization."""

from __future__ import annotations


def normalize_topic(topic: str) -> str:
    topic = str(topic).strip()

    if not topic:
        raise ValueError("Topic name cannot be empty.")

    while "//" in topic:
        topic = topic.replace("//", "/")

    if topic.startswith("~/"):
        return topic.rstrip("/")

    if not topic.startswith("/"):
        topic = f"/{topic}"

    if len(topic) > 1:
        topic = topic.rstrip("/")

    return topic


def join_topic(namespace: str, name: str) -> str:
    namespace = str(namespace).strip()
    name = str(name).strip()

    if not name:
        raise ValueError("Topic name cannot be empty.")

    if name.startswith("/") or name.startswith("~/"):
        return normalize_topic(name)

    if not namespace:
        return normalize_topic(name)

    namespace = normalize_topic(namespace)
    return normalize_topic(f"{namespace}/{name}")


def is_absolute_topic(topic: str) -> bool:
    return str(topic).strip().startswith("/")


def is_private_topic(topic: str) -> bool:
    return str(topic).strip().startswith("~/")


def topic_basename(topic: str) -> str:
    topic = normalize_topic(topic)

    if topic == "/":
        return ""

    return topic.rsplit("/", maxsplit=1)[-1]


def topic_namespace(topic: str) -> str:
    topic = normalize_topic(topic)

    if topic == "/":
        return "/"

    parts = topic.rsplit("/", maxsplit=1)

    if len(parts) == 1 or not parts[0]:
        return "/"

    return parts[0]


def ensure_topic_namespace(topic: str, namespace: str) -> str:
    topic = normalize_topic(topic)
    namespace = normalize_topic(namespace)

    if topic.startswith(f"{namespace}/") or topic == namespace:
        return topic

    return join_topic(namespace, topic_basename(topic))