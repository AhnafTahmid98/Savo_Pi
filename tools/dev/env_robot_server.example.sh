#!/usr/bin/env bash
# PC/Mac running Robot_Savo_Server Docker on the robot hotspot.
ROBOT_SERVER_HOST="192.168.164.119"

ROBOT_LLM_PORT="8000"
ROBOT_STT_PORT="9000"

export ROBOT_ID="robot_savo_pi"

export ROBOT_SERVER_BASE_URL="http://${ROBOT_SERVER_HOST}"

export LLM_SERVER_URL="${ROBOT_SERVER_BASE_URL}:${ROBOT_LLM_PORT}"

export SPEECH_SERVER_URL="${ROBOT_SERVER_BASE_URL}:${ROBOT_STT_PORT}/speech"

echo "[env_robot_server] ROBOT_ID           = ${ROBOT_ID}"
echo "[env_robot_server] ROBOT_SERVER_HOST  = ${ROBOT_SERVER_HOST}"
echo "[env_robot_server] LLM_SERVER_URL     = ${LLM_SERVER_URL}"
echo "[env_robot_server] SPEECH_SERVER_URL  = ${SPEECH_SERVER_URL}"
