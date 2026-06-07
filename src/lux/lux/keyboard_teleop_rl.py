#!/usr/bin/env python3
# flake8: noqa
import select
import sys
import termios
import time
import tty
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from lux_msgs.msg import IMUdata


class RLKeyboardTeleop(Node):
    """Keyboard teleop for lux_rl_interface using /cmd_vel."""

    def __init__(self):
        super().__init__('keyboard_teleop_rl')

        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_vx', 1.0)
        self.declare_parameter('max_vy', 0.35)
        self.declare_parameter('max_wz', 0.8)
        self.declare_parameter('imu_step_deg', 2.0)
        self.declare_parameter('max_roll_deg', 30.0)
        self.declare_parameter('max_pitch_deg', 30.0)

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.imu_step_deg = float(self.get_parameter('imu_step_deg').value)
        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.max_pitch_deg = float(self.get_parameter('max_pitch_deg').value)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.estop = False
        self.last_action = 'Ready'
        self.last_display_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_pub = self.create_publisher(IMUdata, '/spot/imu', 10)
        self.create_timer(1.0 / self.publish_hz, self.publish_state)

        self.settings = termios.tcgetattr(sys.stdin)
        self.display(force=True)

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        return sys.stdin.read(1) if rlist else ''

    @staticmethod
    def clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def zero_motion(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def publish_state(self):
        msg = Twist()
        if not self.estop:
            msg.linear.x = float(self.vx)
            msg.linear.y = float(self.vy)
            msg.angular.z = float(self.wz)
        self.cmd_pub.publish(msg)
        self.imu_pub.publish(self.build_imu_msg())

    def build_imu_msg(self):
        roll = math.radians(self.roll_deg)
        pitch = math.radians(self.pitch_deg)
        gravity = 9.81

        msg = IMUdata()
        msg.roll = float(self.roll_deg)
        msg.pitch = float(self.pitch_deg)
        msg.acc_x = float(-math.sin(pitch) * gravity)
        msg.acc_y = float(math.sin(roll) * math.cos(pitch) * gravity)
        msg.acc_z = float(math.cos(roll) * math.cos(pitch) * gravity)
        msg.gyro_x = 0.0
        msg.gyro_y = 0.0
        msg.gyro_z = 0.0
        return msg

    def display(self, force=False):
        now = time.time()
        if not force and now - self.last_display_time < 0.1:
            return
        self.last_display_time = now

        rst = '\033[0m'
        bold = '\033[1m'
        red = '\033[91m'
        grn = '\033[92m'
        cyn = '\033[96m'
        yel = '\033[93m'

        estop_s = f'{red}E-STOP{rst}' if self.estop else f'{grn}RUN{rst}'
        out = '\r\033[H\033[J'
        out += f'{bold}{cyn}LUX RL KEYBOARD TELEOP{rst}  {estop_s}\r\n'
        out += f'{yel}/cmd_vel{rst}  vx:{self.vx:+.2f}  vy:{self.vy:+.2f}  wz:{self.wz:+.2f}\r\n\r\n'
        out += f'{yel}/spot/imu{rst}  roll:{self.roll_deg:+.1f}deg  pitch:{self.pitch_deg:+.1f}deg\r\n\r\n'
        out += '  w/x : vx +/-    a/d : vy +/-    q/e : yaw +/-\r\n'
        out += '  i/k : pitch +/-    j/l : roll +/-    b : IMU reset\r\n'
        out += '  s or SPACE : stop motion    0 : E-Stop toggle    ESC/Ctrl-C : quit\r\n\r\n'
        out += f'  LOG: {self.last_action}\r\n'

        sys.stdout.write(out)
        sys.stdout.flush()

    def handle_key(self, key):
        if key == '0':
            self.estop = not self.estop
            self.zero_motion()
            self.last_action = 'E-Stop ON' if self.estop else 'E-Stop OFF'
            return

        if self.estop:
            self.last_action = 'E-Stop active'
            return

        if key == 'w':
            self.vx = self.clamp(self.vx + self.linear_step, 0.0, self.max_vx)
            self.last_action = 'Forward'
        elif key == 'x':
            self.vx = self.clamp(self.vx - self.linear_step, 0.0, self.max_vx)
            self.last_action = 'Forward down'
        elif key == 'a':
            self.vy = self.clamp(self.vy + self.linear_step, -self.max_vy, self.max_vy)
            self.last_action = 'Left'
        elif key == 'd':
            self.vy = self.clamp(self.vy - self.linear_step, -self.max_vy, self.max_vy)
            self.last_action = 'Right'
        elif key == 'q':
            self.wz = self.clamp(self.wz + self.angular_step, -self.max_wz, self.max_wz)
            self.last_action = 'Yaw left'
        elif key == 'e':
            self.wz = self.clamp(self.wz - self.angular_step, -self.max_wz, self.max_wz)
            self.last_action = 'Yaw right'
        elif key in ('s', ' '):
            self.zero_motion()
            self.last_action = 'Stop'
        elif key == 'i':
            self.pitch_deg = self.clamp(
                self.pitch_deg + self.imu_step_deg,
                -self.max_pitch_deg,
                self.max_pitch_deg,
            )
            self.last_action = f'Pitch {self.pitch_deg:+.1f} deg'
        elif key == 'k':
            self.pitch_deg = self.clamp(
                self.pitch_deg - self.imu_step_deg,
                -self.max_pitch_deg,
                self.max_pitch_deg,
            )
            self.last_action = f'Pitch {self.pitch_deg:+.1f} deg'
        elif key == 'j':
            self.roll_deg = self.clamp(
                self.roll_deg + self.imu_step_deg,
                -self.max_roll_deg,
                self.max_roll_deg,
            )
            self.last_action = f'Roll {self.roll_deg:+.1f} deg'
        elif key == 'l':
            self.roll_deg = self.clamp(
                self.roll_deg - self.imu_step_deg,
                -self.max_roll_deg,
                self.max_roll_deg,
            )
            self.last_action = f'Roll {self.roll_deg:+.1f} deg'
        elif key == 'b':
            self.roll_deg = 0.0
            self.pitch_deg = 0.0
            self.last_action = 'IMU reset'

    def run(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in ('\x1b', '\x03'):
                    break
                if key:
                    self.handle_key(key)
                    self.display(force=True)
                else:
                    self.display()
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            self.zero_motion()
            self.publish_state()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = RLKeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
