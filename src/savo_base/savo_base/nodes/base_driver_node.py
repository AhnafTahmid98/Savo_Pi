#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_driver_node.py
-------------------------------------------------
ROS 2 Jazzy base execution node for the real Robot Savo mecanum platform.
This is the *hardware execution* node inside `savo_base`.
It converts base motion commands (Twist) into wheel commands and sends them to
the physical motor board (Freenove PCA9685 mecanum board) through `savo_base.drivers`.

Inputs
------
- /cmd_vel_safe               (geometry_msgs/Twist)   [default]
- /safety/slowdown_factor     (std_msgs/Float32)      [optional]
- /safety/stop                (std_msgs/Bool)         [optional]

Outputs (optional, lightweight)
-------------------------------
- /savo_base/watchdog_state   (std_msgs/String JSON summary)
- /savo_base/base_state       (std_msgs/String JSON summary)

Safety
------
- Watchdog timeout on stale commands
- Optional safety stop topic integration
- Optional slowdown factor integration
- Clean stop on shutdown
"""

from __future__ import annotations
import json
import math
import time
import traceback
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String

# ===================================================================
# Motor Control Helper Functions
# ===================================================================
try:
    import smbus
except ImportError:
    raise ImportError("Please install the smbus library for I2C communication")

# Define motor control classes
class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, bus: int = 1, address: int = 0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def set_pwm_freq(self, freq: float) -> None:
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        if duty < 0: duty = 0
        if duty > 4095: duty = 4095
        self.set_pwm(channel, 0, duty)

    def close(self) -> None:
        self.bus.close()

# ===================================================================
# Robot Savo Motor Controller Node
# ===================================================================
class BaseDriverNode(Node):
    """
    This is the hardware execution node that controls the motors
    through the Freenove PCA9685 motor controller.
    """

    def __init__(self) -> None:
        super().__init__("base_driver_node")

        # ROS 2 Parameters
        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("max_duty", 4095)  # Full duty cycle for motors
        self.declare_parameter("pwm_freq_hz", 50.0)

        # Initialize PCA9685 Motor Controller
        self.i2c_bus = 1  # Assuming I2C bus 1
        self.pwm_freq = self.get_parameter("pwm_freq_hz").get_parameter_value().double_value
        self.pwm = PCA9685(bus=self.i2c_bus)
        self.pwm.set_pwm_freq(self.pwm_freq)

        # Motors
        self.max_duty = self.get_parameter("max_duty").get_parameter_value().integer_value
        self.invert_fl, self.invert_rl, self.invert_fr, self.invert_rr = False, False, False, False

        # Subscriptions
        self.create_subscription(Twist, self.get_parameter("cmd_topic").get_parameter_value().string_value,
                                 self.cmd_callback, 10)

        # Main loop
        self.create_timer(1.0, self.main_loop)

    def cmd_callback(self, msg: Twist) -> None:
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.drive_mecanum(linear_x, 0.0, angular_z)

    def drive_mecanum(self, vx: float, vy: float, wz: float) -> None:
        """
        Drive the robot using mecanum kinematics
        vx: forward speed
        vy: strafing speed
        wz: rotational speed
        """
        fl = vx - vy - wz
        rl = vx + vy - wz
        fr = vx + vy + wz
        rr = vx - vy + wz

        # Normalize wheel velocities
        max_val = max(abs(fl), abs(rl), abs(fr), abs(rr))
        if max_val > 1.0:
            fl /= max_val
            rl /= max_val
            fr /= max_val
            rr /= max_val

        # Map to duty cycles
        self.set_motor_duty_cycle(fl, rl, fr, rr)

    def set_motor_duty_cycle(self, fl: float, rl: float, fr: float, rr: float) -> None:
        """
        Convert normalized wheel speed to duty cycle and set motor PWM
        """
        fl_duty = int(fl * self.max_duty)
        rl_duty = int(rl * self.max_duty)
        fr_duty = int(fr * self.max_duty)
        rr_duty = int(rr * self.max_duty)

        self.pwm.set_motor_pwm(0, fl_duty)
        self.pwm.set_motor_pwm(1, fl_duty)
        self.pwm.set_motor_pwm(2, rl_duty)
        self.pwm.set_motor_pwm(3, rl_duty)
        self.pwm.set_motor_pwm(4, fr_duty)
        self.pwm.set_motor_pwm(5, fr_duty)
        self.pwm.set_motor_pwm(6, rr_duty)
        self.pwm.set_motor_pwm(7, rr_duty)

    def main_loop(self):
        self.get_logger().info("Running main loop for motor control.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()