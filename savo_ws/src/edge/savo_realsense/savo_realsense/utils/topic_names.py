# Copyright 2026 Ahnaf Tahmid
def normalize_topic_name(topic: str) -> str:
    cleaned = topic.strip()
    if not cleaned:
        raise ValueError("Topic name cannot be empty")

    if not cleaned.startswith("/"):
        cleaned = f"/{cleaned}"

    while "//" in cleaned:
        cleaned = cleaned.replace("//", "/")

    return cleaned


def join_topic(namespace: str, *parts: str) -> str:
    cleaned_namespace = normalize_topic_name(namespace)

    cleaned_parts = []
    for part in parts:
        stripped = part.strip("/")
        if stripped:
            cleaned_parts.append(stripped)

    if not cleaned_parts:
        return cleaned_namespace

    return normalize_topic_name("/".join([cleaned_namespace, *cleaned_parts]))
