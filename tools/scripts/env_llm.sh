#!/usr/bin/env bash
# Robot Savo — LLM server config for the Pi
# This is the ONLY file you edit when the LLM server IP/port changes.

# Replace this IP with the hotspot IP of your LLM server (PC or MacBook)
export LLM_SERVER_URL="http://10.208.250.119:8000"

echo "[env_llm] LLM_SERVER_URL = ${LLM_SERVER_URL}"
 