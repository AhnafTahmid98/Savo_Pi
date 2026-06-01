#!/usr/bin/env bash
#
# Robot Savo – Robot_Savo_Server environment (STT + LLM gateway)
# Use this on the Pi:
#   cd ~/Savo_Pi
#   source tools/scripts/env_robot_server.sh

#############################
# EDIT THIS PART FOR YOUR PC
#############################

# IP or hostname of the machine running Robot_Savo_Server Docker
# Example: "192.168.164.119"  (replace with your real PC/Mac IP)
ROBOT_SERVER_HOST="192.168.164.119"

# Robot_Savo_Server ports (default from docker-compose)
ROBOT_LLM_PORT="8000"    # robot-llm container (FastAPI /chat, /health, etc.)
ROBOT_STT_PORT="9000"    # robot-stt container (/speech endpoint)

#############################
# CONSTANTS / DEFAULTS
#############################

# Robot ID used in logs / IntentResult
export ROBOT_ID="robot_savo_pi"

# Base URLs
export ROBOT_SERVER_BASE_URL="http://${ROBOT_SERVER_HOST}"

# LLM server URL (if you still use intent_client_node or /chat tests)
export LLM_SERVER_URL="${ROBOT_SERVER_BASE_URL}:${ROBOT_LLM_PORT}"

# Speech server URL (Pi → /speech for STT+LLM)
export SPEECH_SERVER_URL="${ROBOT_SERVER_BASE_URL}:${ROBOT_STT_PORT}/speech"

#############################
# ECHO FOR DEBUG
#############################

echo "[env_robot_server] ROBOT_ID           = ${ROBOT_ID}"
echo "[env_robot_server] ROBOT_SERVER_HOST  = ${ROBOT_SERVER_HOST}"
echo "[env_robot_server] LLM_SERVER_URL     = ${LLM_SERVER_URL}"
echo "[env_robot_server] SPEECH_SERVER_URL  = ${SPEECH_SERVER_URL}"
