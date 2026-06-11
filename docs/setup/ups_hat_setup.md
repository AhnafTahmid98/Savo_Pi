# UPS HAT Setup

This document describes the UPS HAT setup used on the Robot Savo Raspberry Pi 5 units.

The same UPS HAT setup procedure applies to both `savo-core` and `savo-edge` when the same Raspberry Pi 5 UPS HAT style is installed. Component validation is documented separately because each Pi has different hardware responsibilities.

## Role in Robot Savo

The UPS HAT provides stable Raspberry Pi power, battery backup, voltage monitoring, and power-loss visibility. It is part of the robot power layer and should be validated before higher-level ROS 2 services are started.

For Robot Savo, the UPS HAT is used for:

* stable Raspberry Pi 5 power delivery
* battery voltage and percentage monitoring
* AC adapter / power-loss status checks
* Raspberry Pi 5 power configuration validation
* fan and system health monitoring

## Power connection

The Raspberry Pi must be powered through the UPS HAT, not directly through the Raspberry Pi USB-C port.

Correct power path:

```text
Power adapter
    ↓
UPS HAT input
    ↓
UPS HAT powers Raspberry Pi through GPIO header
```

Do not use the Pi USB-C port as the normal power input when testing UPS behavior. If the adapter is connected directly to the Pi USB-C port, the UPS HAT may still report some information, but the Pi is not being powered through the UPS power path.

## Tested platform

The first validation was completed on `savo-edge`.

```text
Host: edge
User: savo
OS: Ubuntu 24.04 LTS
Board: Raspberry Pi 5
UPS monitor scripts: /opt/x120x
UPS I2C address: 0x36
```

The same setup is intended for `savo-core` when the same UPS HAT is installed.

## Install required packages

Install the base tools:

```bash
sudo apt update
sudo apt install -y git i2c-tools python3-pip python3-smbus python3-venv
```

Install the Python dependencies required by the UPS scripts:

```bash
sudo apt install -y python3-smbus2 python3-gpiozero python3-lgpio
```

These packages are needed for:

* I²C communication with the UPS fuel gauge
* GPIO-based power status detection
* terminal-based UPS monitoring

## Validate I2C

Confirm that I²C bus 1 exists:

```bash
ls /dev/i2c-1
```

Scan the I²C bus:

```bash
sudo i2cdetect -y 1
```

Expected result:

```text
30: -- -- -- -- -- -- 36 -- -- -- -- -- -- -- -- --
```

The UPS fuel gauge should appear at:

```text
0x36
```

If `0x36` is not visible, check the UPS HAT seating, Raspberry Pi GPIO connection, and I²C configuration before continuing.

## Install UPS helper scripts

Clone the X120x helper scripts under `/opt`:

```bash
cd /opt
sudo git clone https://github.com/suptronics/x120x.git
sudo chown -R savo:savo /opt/x120x
cd /opt/x120x
```

Check available scripts:

```bash
ls
find /opt/x120x -maxdepth 2 -type f -name "*.py" -print
```

Expected useful scripts include:

```text
bat.py
qtx120xTerminal.py
pld.py
merged.py
```

The repository may not contain a file named `x120x.py`. Use the scripts that exist in `/opt/x120x`.

## Test battery reading

Run:

```bash
python3 /opt/x120x/bat.py
```

Expected output style:

```text
******************
Voltage: 4.16V
Battery:   97%
```

Stop the script with:

```text
CTRL + C
```

This confirms that the UPS battery monitor is readable over I²C.

## Test terminal UPS monitor

Run:

```bash
python3 /opt/x120x/qtx120xTerminal.py
```

Expected output style:

```text
========== X120x UPS Status ==========
UPS Voltage: 4.197V
Battery: 97.500%
Charging: disabled

========== RPi5 System Stats ==========
Input Voltage: 5.096V
CPU Volts: 0.859V
CPU Amps: 1.259A
System Watts: 2.500W
CPU Temp: 47.7°C
Fan RPM: 2528 RPM

========== Power Status ==========
AC Power: OK! | Power Adapter: OK!
======================================
```

Stop the monitor with:

```text
CTRL + C
```

`Charging: disabled` is not automatically a fault when the battery is nearly full. During validation, the battery was around 97%, so charging may be paused by the UPS board logic.

## Raspberry Pi 5 EEPROM power configuration

Raspberry Pi 5 may restrict peripheral power if it does not detect a suitable 5A-capable power source. Because the Pi is powered through the UPS HAT, configure the EEPROM so the Pi uses the correct 5A power profile.

Check the current EEPROM config:

```bash
sudo rpi-eeprom-config
```

Edit the EEPROM config:

```bash
sudo -E rpi-eeprom-config --edit
```

Use this configuration:

```text
[all]
BOOT_UART=1
POWER_OFF_ON_HALT=1
BOOT_ORDER=0xf461
PSU_MAX_CURRENT=5000
```

After saving, the update should complete with:

```text
VERIFY: SUCCESS
UPDATE SUCCESSFUL
```

Reboot:

```bash
sudo reboot
```

After reboot, verify:

```bash
sudo rpi-eeprom-config | grep -E "PSU_MAX_CURRENT|POWER_OFF_ON_HALT"
```

Expected result:

```text
POWER_OFF_ON_HALT=1
PSU_MAX_CURRENT=5000
```

## Raspberry Pi 5 fan profile

Back up the firmware config before editing:

```bash
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak.$(date +%F-%H%M)
```

Edit the config:

```bash
sudo nano /boot/firmware/config.txt
```

Add this under the final `[all]` section:

```ini
# Robot Savo edge — Raspberry Pi 5 active cooler profile
# Values are in millicelsius: 45000 = 45°C.
# Fan speed range is 0–255.

dtparam=cooling_fan=on

# Start the fan gently when the Pi reaches 45°C.
# Hysteresis prevents constant on/off switching near the threshold.
dtparam=fan_temp0=45000
dtparam=fan_temp0_hyst=5000
dtparam=fan_temp0_speed=80

# Increase fan speed at 55°C for normal edge workloads.
dtparam=fan_temp1=55000
dtparam=fan_temp1_hyst=5000
dtparam=fan_temp1_speed=140

# Use high fan speed at 65°C for heavier RealSense, VO, or Docker load.
dtparam=fan_temp2=65000
dtparam=fan_temp2_hyst=5000
dtparam=fan_temp2_speed=200

# Full fan speed at 75°C to protect the Pi under sustained high load.
dtparam=fan_temp3=75000
dtparam=fan_temp3_hyst=5000
dtparam=fan_temp3_speed=255
```

Save and exit:

```text
CTRL + O
Enter
CTRL + X
```

Reboot:

```bash
sudo reboot
```

After reboot, run:

```bash
python3 /opt/x120x/qtx120xTerminal.py
```

The monitor should show CPU temperature and fan RPM when the fan is active.

## Related diagnostic scripts

Robot Savo keeps power-related diagnostic scripts under:

```text
tools/diag/power/
```

Current scripts:

```text
battery_health.py
current_draw_logger.py
power_battery.py
system_health.py
```

These scripts are used for power and system health checks before long-running robot services are enabled.

## Validation checklist

Run these commands after setup:

```bash
hostname
whoami
sudo i2cdetect -y 1
sudo rpi-eeprom-config | grep -E "PSU_MAX_CURRENT|POWER_OFF_ON_HALT"
python3 /opt/x120x/qtx120xTerminal.py
```

Expected results:

```text
hostname: edge or core
whoami: savo
I2C: 0x36 visible
POWER_OFF_ON_HALT=1
PSU_MAX_CURRENT=5000
UPS monitor shows battery, voltage, input power, AC status, CPU temperature, and fan RPM
```

## Validation evidence

The following screenshots were captured on `savo-edge` after the UPS HAT setup was completed.

### Pi 5 fan profile

![Pi 5 fan profile](../assets/hardware/pi5_fan_profile.png)

This screenshot shows the Raspberry Pi 5 active cooler profile added under `/boot/firmware/config.txt`. The fan profile starts at 45°C and increases fan speed at 55°C, 65°C, and 75°C.

### UPS HAT validation

![UPS HAT validation](../assets/hardware/ups_hat.png)

This screenshot confirms:

* UPS HAT detected on I²C bus 1 at `0x36`
* EEPROM power settings are correct:

  * `POWER_OFF_ON_HALT=1`
  * `PSU_MAX_CURRENT=5000`
* UPS monitor is working through `/opt/x120x/qtx120xTerminal.py`
* Battery percentage, UPS voltage, input voltage, CPU temperature, fan RPM, AC power, and power adapter status are visible

## Notes

* `savo-core` and `savo-edge` can use the same UPS HAT setup procedure.
* Component validation should still be recorded separately for each Pi because the attached hardware is different.
* Do not enable automatic shutdown until AC power-loss behavior has been tested safely.
* For normal operation, power the Raspberry Pi through the UPS HAT input path.
* Keep screenshots under `docs/assets/hardware/` so setup documents can reference them cleanly.
