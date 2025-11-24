# Robot Savo — Result from LLM /chat
# ----------------------------------
# High-level output from the LLM server, already parsed and cleaned.
# This is what other ROS2 nodes (Nav2, UI, TTS, etc.) should subscribe to.

# What Robot Savo should actually say out loud.
# Short, B1 English sentences, already clamped for length.
string reply_text

# High-level intent decided by the deterministic classifier:
# One of: "STOP", "FOLLOW", "NAVIGATE", "STATUS", "CHATBOT"
string intent

# Canonical navigation goal if this is a NAVIGATE or FOLLOW intent.
# Examples: "A201", "Info Desk". Empty string ("") when not applicable.
string nav_goal

# Optional metadata for debugging / logging.
# session_id ties this message back to one conversation on the LLM server.
string session_id

# Which tier was used by the LLM server: "tier1", "tier2", "tier3".
string tier_used
