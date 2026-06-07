#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math

import rclpy
from rclpy.node import Node

import busio
import board
from adafruit_pca9685 import PCA9685
from lux_msgs.msg import JointAngles, JointPulse


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PWMPubDualBoard(Node):
    def __init__(self):
        super().__init__("pwm_publisher_dual_board")

        self.declare_parameter("freq", 50)
        self.declare_parameter("pulse_min", 500)
        self.declare_parameter("pulse_max", 2500)
        self.declare_parameter("max_vel_bypass", 5.0)

        self.freq = self.get_parameter("freq").value
        self.pulse_min = self.get_parameter("pulse_min").value
        self.pulse_max = self.get_parameter("pulse_max").value
        self.MAX_VEL_BYPASS = self.get_parameter("max_vel_bypass").value

        self.pulse_mid = (self.pulse_min + self.pulse_max) / 2.0
        self.upr_default = 636.0

        self.target_run_vel = 6.0
        self.max_velocity_rad_s = 0.5
        self.startup_mode = True
        self.internal_setpoint = [0.0] * 12
        self.last_update_time = time.time()
        self.channels = list(range(12))

        try:
            i2c = busio.I2C(board.SCL, board.SDA)

            self.pca0 = PCA9685(i2c, address=0x40)
            self.pca0.frequency = self.freq

            self.pca1 = PCA9685(i2c, address=0x41)
            self.pca1.frequency = self.freq

            self.motor_map = {
                0:  (self.pca0, 0),
                1:  (self.pca0, 1),
                2:  (self.pca0, 2),
                3:  (self.pca0, 3),
                4:  (self.pca0, 4),
                5:  (self.pca0, 5),
                6:  (self.pca1, 0),
                7:  (self.pca1, 1),
                8:  (self.pca1, 2),
                9:  (self.pca1, 3),
                10: (self.pca1, 4),
                11: (self.pca1, 5),
            }

            self.get_logger().info("Dual PCA9685 Initialized (0x40, 0x41)")
        except Exception as e:
            self.get_logger().error(f"Dual PCA9685 Init Failed: {e}")
            sys.exit(1)

        self.create_subscription(JointAngles, "/spot/joints_cal", self.cb_joint, 1)
        self.create_subscription(JointPulse, "/spot/pulse", self.cb_pulse, 1)

    def write_pos_immediate(self, ch, rad_val):
        pulse_us = int(round(self.pulse_mid + self.upr_default * rad_val))
        pulse_us = clamp(pulse_us, self.pulse_min, self.pulse_max)

        period_us = int(1e6 / self.freq)
        duty = int(65535 * (float(pulse_us) / period_us))

        if ch not in self.motor_map:
            return

        target_pca, target_channel = self.motor_map[ch]
        target_pca.channels[target_channel].duty_cycle = clamp(duty, 0, 65535)

    def cb_joint(self, msg):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        if dt > 0.1:
            dt = 0.01

        qraw = [msg.fls, msg.fle, msg.flw, msg.frs, msg.fre, msg.frw,
                msg.bls, msg.ble, msg.blw, msg.brs, msg.bre, msg.brw]

        targets_rad = [math.radians(v) for v in qraw]

        if self.max_velocity_rad_s >= self.MAX_VEL_BYPASS:
            for i in range(12):
                self.internal_setpoint[i] = targets_rad[i]
                self.write_pos_immediate(i, targets_rad[i])
        else:
            max_diff = 0.0
            diffs = [0.0] * 12

            for i in range(12):
                diff = targets_rad[i] - self.internal_setpoint[i]
                diffs[i] = diff
                if abs(diff) > max_diff:
                    max_diff = abs(diff)

            if self.startup_mode and max_diff < 0.05:
                self.max_velocity_rad_s = self.target_run_vel
                self.startup_mode = False

            max_step = self.max_velocity_rad_s * dt
            scale = min(1.0, max_step / max_diff) if max_diff > 1e-6 else 1.0

            for i in range(12):
                self.internal_setpoint[i] += diffs[i] * scale
                self.write_pos_immediate(i, self.internal_setpoint[i])

    def cb_pulse(self, jp):
        try:
            mid, val = int(jp.servo_id), float(jp.servo_deg)
            if mid == 99:
                if val > 0.1:
                    self.max_velocity_rad_s = val
            elif mid in self.channels:
                target_rad = math.radians(val)
                self.write_pos_immediate(mid, target_rad)
                self.internal_setpoint[mid] = target_rad
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PWMPubDualBoard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
