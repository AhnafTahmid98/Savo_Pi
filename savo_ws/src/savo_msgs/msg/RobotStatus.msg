# Robot Savo — Robot Status
# Snapshot of the robot's health and runtime state.
# This will be serialized to JSON and sent to the LLM server.

std_msgs/Header header

# Logical ID of this robot (e.g. "robot_savo_pi")
string robot_id

# -------------------------------------------------------------------
# Battery / power status
# -------------------------------------------------------------------
# UPS HAT battery (the Pi power source)
# Use -1.0 when value is unknown / not measured.
float32 ups_voltage_v         # e.g. 3.7–4.2 V
float32 ups_soc_pct           # 0–100 %, SoC estimate

# Kit (drive) battery powering motors and sensors
float32 kit_voltage_v         # e.g. 6.4–8.4 V range
float32 kit_soc_pct           # 0–100 %, SoC estimate

# High-level battery health summary (words only):
#  - "UPS low"
#  - "Kit low"
#  - "Both low"
#  - "Good condition"
#  - "Needs charging"
string battery_health_note

# True when any battery is considered critically low for safe navigation.
bool battery_critical

# -------------------------------------------------------------------
# Thermal status
# -------------------------------------------------------------------
# Temperatures in °C. Use NaN or -1.0 if a sensor is missing.
float32 cpu_temp_c            # CPU temperature from Pi
float32 board_temp_c          # Optional: other board/internal sensor

# True if any temperature is in a "too hot / throttling risk" range.
bool overheating

# -------------------------------------------------------------------
# Motion / system state
# -------------------------------------------------------------------
bool motors_enabled           # Motor drivers enabled (can move)
bool in_motion                # Robot is actually moving now
bool estop_active             # Emergency stop asserted (near-field or physical)

# -------------------------------------------------------------------
# Connectivity / uptime
# -------------------------------------------------------------------
bool network_ok               # True if Wi-Fi/network is working
float32 wifi_rssi_dbm         # Wi-Fi signal strength (dBm); use 0.0 or -100.0 if unknown

# Robot uptime in seconds since last boot.
float32 uptime_sec

# -------------------------------------------------------------------
# Diagnostics / notes
# -------------------------------------------------------------------
# Last significant error code or short label (e.g. "LIDAR_TIMEOUT", "IMU_FAIL").
string last_error_code

# Free-form human-readable status line, e.g.:
# "All systems nominal", "Mapping mode, battery 40%", "Waiting for network".
string status_note
