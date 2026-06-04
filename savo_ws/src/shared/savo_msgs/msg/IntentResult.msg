# Robot Savo — Result from LLM /chat
# ----------------------------------
# High-level output from the LLM server, already parsed and cleaned.
# This is what other ROS 2 nodes (Nav2, UI, TTS, safety, etc.) should subscribe to.

# ---------------------------------------------------------------------------
# Source / identity
# ---------------------------------------------------------------------------

# Where this text came from in the robot:
# - "system"   : internal call from the ROS2 pipeline
# - "mic"      : voice input from microphone
# - "keyboard" : manually typed on a UI
# - "test"     : test tools / scripts
string source

# Which robot produced this result (e.g. "robot_savo_pi").
string robot_id

# ---------------------------------------------------------------------------
# Text content
# ---------------------------------------------------------------------------

# What the human said (after STT / preprocessing).
string user_text

# What Robot Savo should actually say out loud.
# Short, B1 English sentences, already clamped for length on the server side.
string reply_text

# ---------------------------------------------------------------------------
# Intent and navigation decision
# ---------------------------------------------------------------------------

# High-level intent decided by the deterministic classifier:
# One of: "STOP", "FOLLOW", "NAVIGATE", "STATUS", "CHATBOT"
string intent

# Canonical navigation goal if this is a NAVIGATE or FOLLOW intent.
# Examples: "A201", "Info Desk". Empty string ("") when not applicable.
string nav_goal

# ---------------------------------------------------------------------------
# LLM backend info
# ---------------------------------------------------------------------------

# Which tier was used by the LLM server:
# - "tier1"  : online OpenRouter (primary)
# - "tier2"  : local Ollama
# - "tier3"  : Pi fallback templates
# - "tier_error" : used when the bridge had to fall back locally
string tier_used

# ---------------------------------------------------------------------------
# Health / diagnostics
# ---------------------------------------------------------------------------

# True if the /chat call succeeded end-to-end and produced a valid reply_text.
bool success

# Short error description when success == false, otherwise empty string ("").
# Examples: "timeout", "http_error", "url_error", "json_error"
string error
