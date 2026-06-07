#!/usr/bin/env python3
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from lux_msgs.msg import IMUdata, JointAngles, JointPulse, JoyButtons, MiniCmd


class SpotKeyboardTeleop(Node):
    JOINT_NAMES = ['FLS', 'FLE', 'FLW', 'FRS', 'FRE', 'FRW',
                   'BLS', 'BLE', 'BLW', 'BRS', 'BRE', 'BRW']

    def __init__(self):
        super().__init__('keyboard_teleop_spot')

        self.scale = 0.5
        self.vx = self.vy = self.wz = 0.0
        self.roll = self.pitch = self.yaw = self.z = 0.0
        self.rpy_step = 0.05
        self.current_max_vel = 1.0

        self.motion = 'Stop'
        self.movement = 'Stepping'
        self.pose_cmd = 'Normal'
        self.imu_auto_pose = False
        self.estop = False
        self.last_action = 'Ready'

        self.imu_roll = self.imu_pitch = self.imu_yaw = 0.0
        self.imu_gyro = [0.0, 0.0, 0.0]
        self.imu_acc = [0.0, 0.0, 0.0]
        self.imu_fresh = False

        self.joint_raw = [0.0] * 12

        self.declare_parameter('calibration.direction', [1] * 12)
        self.declare_parameter('calibration.offset_deg', [0.0] * 12)
        self.declare_parameter('calibration.neutral_us', [1500] * 12)
        self.declare_parameter('calibration.us_per_rad', 636.0)

        self.servo_dir = list(self.get_parameter('calibration.direction').value)
        self.servo_offset = list(self.get_parameter('calibration.offset_deg').value)
        self.servo_neutral_us = list(self.get_parameter('calibration.neutral_us').value)
        self.servo_us_per_rad = float(self.get_parameter('calibration.us_per_rad').value)

        self.cmd_pub = self.create_publisher(MiniCmd, '/mini_cmd', 20)
        self.jb_pub = self.create_publisher(JoyButtons, '/joybuttons', 20)
        self.pulse_pub = self.create_publisher(JointPulse, '/spot/pulse', 10)

        self.create_subscription(MiniCmd, '/mini_cmd', self._cb_mini_cmd, 10)
        self.create_subscription(IMUdata, '/spot/imu', self._cb_imu, 10)
        self.create_subscription(JointAngles, '/spot/joints', self._cb_joints_raw, 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.last_display_time = 0.0
        os.system('clear')

    def _cb_mini_cmd(self, msg):
        self.motion = msg.motion
        self.movement = msg.movement
        self.vx = msg.x_velocity
        self.vy = msg.y_velocity
        self.wz = msg.rate
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
        self.z = msg.z
        self.pose_cmd = msg.pose_cmd
        self.imu_auto_pose = msg.imu_auto_pose

    def _cb_imu(self, msg):
        try:
            self.imu_roll = msg.roll
            self.imu_pitch = msg.pitch
            self.imu_yaw = getattr(msg, 'yaw', 0.0)
            self.imu_gyro = [msg.gyro_x, msg.gyro_y, msg.gyro_z]
            self.imu_acc = [msg.acc_x, msg.acc_y, msg.acc_z]
            self.imu_fresh = True
        except Exception:
            pass

    def _cb_joints_raw(self, msg):
        self.joint_raw = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        return sys.stdin.read(1) if rlist else ''

    def send_pulse(self, val):
        self.current_max_vel = max(0.5, min(20.0, self.current_max_vel + val))
        pulse = JointPulse()
        pulse.servo_id = 99
        pulse.servo_deg = float(self.current_max_vel)
        self.pulse_pub.publish(pulse)
        self.last_action = f'MaxVel: {self.current_max_vel:.1f} r/s'

    def display_menu(self, force=False):
        now = time.time()
        if not force and (now - self.last_display_time < 0.1):
            return
        self.last_display_time = now

        rst = '\033[0m'
        bold = '\033[1m'
        dim = '\033[2m'
        red = '\033[91m'
        grn = '\033[92m'
        yel = '\033[93m'
        cyn = '\033[96m'
        width = 90

        estop_s = f'{red}● E-STOP{rst}' if self.estop else f'{grn}○ 정상{rst}'
        mode_s = f'{yel}VIEWING{rst}' if self.movement == 'Viewing' else f'{cyn}STEPPING{rst}'
        mot_s = f'{grn}Go{rst}' if self.motion == 'Go' else f'{yel}Stop{rst}'
        imu_s = f'{yel}ON{rst}' if self.imu_auto_pose else 'OFF'
        panel_s = f'{cyn}[SPOT]{rst}'

        out = '\r\033[H\033[J'
        out += f'{bold}{"=" * width}{rst}\r\n'
        out += f'{bold}{cyn}  LUX DEBUG CONSOLE{rst}  {panel_s}'
        out += f'   {mode_s} | {mot_s} | Scale:{self.scale:.2f} | IMU:{imu_s} | {estop_s}\r\n'
        out += f'{bold}{"=" * width}{rst}\r\n'

        out += f'{yel}[ CMD  /mini_cmd ]{rst}\r\n'
        out += f'  Motion:{self.motion:<8} Movement:{self.movement:<12} Pose:{self.pose_cmd}\r\n'
        out += (f'  vx:{self.vx:+.2f}  vy:{self.vy:+.2f}  wz:{self.wz:+.2f}'
                f'    Roll:{self.roll:+.2f}  Pitch:{self.pitch:+.2f}'
                f'  Yaw:{self.yaw:+.2f}  Z:{self.z:+.2f}\r\n\r\n')

        imu_color = grn if self.imu_fresh else ''
        out += f'{yel}[ IMU  /spot/imu ]{rst}\r\n'
        out += (f'{imu_color}  Roll:{self.imu_roll:+7.2f}  Pitch:{self.imu_pitch:+7.2f}'
                f'  Yaw:{self.imu_yaw:+7.2f}  (deg)\r\n{rst}')
        out += (f'{imu_color}  Gyro:[{self.imu_gyro[0]:+6.2f} {self.imu_gyro[1]:+6.2f}'
                f' {self.imu_gyro[2]:+6.2f}] deg/s'
                f'    Acc:[{self.imu_acc[0]:+6.2f} {self.imu_acc[1]:+6.2f}'
                f' {self.imu_acc[2]:+6.2f}] m/s2\r\n{rst}\r\n')

        out += self._render_spot_panel(yel, rst)
        out += self._render_key_reference(yel, cyn, dim, rst, width)
        out += f'{bold}{"-" * width}{rst}\r\n'
        out += f'  LOG: {self.last_action}\r\n'
        out += f'{bold}{"=" * width}{rst}\r\n'

        sys.stdout.write(out)
        sys.stdout.flush()

    def _render_spot_panel(self, yel, rst):
        out = f'{yel}[ SPOT SERVOS  /spot/joints (raw IK) + servo_calib ]{rst}\r\n'
        out += (f'  {"Joint":<5} {"ID":>2}  {"Target°":>8}  {"Calib°":>8}  {"Offset°":>8}'
                f'  {"Dir":>4}  {"Neutral":>8}  {"Pulse(us)":>9}\r\n')
        out += f'  {"-" * 82}\r\n'
        for i in range(12):
            target_deg = self.joint_raw[i]
            cal_deg = self.servo_dir[i] * (target_deg + self.servo_offset[i])
            pulse_us = int(round(
                self.servo_neutral_us[i] + self.servo_us_per_rad * math.radians(cal_deg)
            ))
            pulse_us = max(500, min(2500, pulse_us))
            out += (f'  {self.JOINT_NAMES[i]:<5} {i:>2}'
                    f'  {target_deg:>8.2f}'
                    f'  {cal_deg:>8.2f}'
                    f'  {self.servo_offset[i]:>8.2f}'
                    f'  {self.servo_dir[i]:>4d}'
                    f'  {self.servo_neutral_us[i]:>8d}'
                    f'  {pulse_us:>9d}\r\n')
        out += '\r\n'
        return out

    def _render_key_reference(self, yel, cyn, dim, rst, width):
        sep = f'{dim}{"─" * (width - 4)}{rst}'
        out = f'{yel}[ KEY REFERENCE ]{rst}\r\n'
        out += f'  {sep}\r\n'
        out += f'  {cyn}이동{rst}      w/x 전후진  a/d 좌우  q/e 회전  s 이동정지  SPACE 전체정지  0 E-Stop\r\n'
        out += f'  {cyn}자세{rst}      i/k Pitch   j/l Roll  u/o Yaw   r/f 높이     b 자세초기화   m IMU토글\r\n'
        out += f'  {cyn}파라미터{rst}  t/g 발높이   [/] 스케일  +/- 최대속도\r\n'
        out += f'  {cyn}특수자세{rst}  3 Sit  c Zero  5 0deg   v 모드전환(Stepping↔Viewing)  p Motion GO\r\n'
        out += f'  {sep}\r\n'
        return out

    def run(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()

                if key:
                    if key in ('\x1b', '\x03'):
                        break

                    if key == '0':
                        self.estop = not self.estop
                        self.vx = self.vy = self.wz = 0.0
                        self.motion = 'Stop'
                        self.last_action = 'EMERGENCY STOP'

                    if not self.estop:
                        moved = False

                        if key == 'w':
                            self.vx = 0.0 if self.vx == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Fwd {'ON' if self.vx else 'OFF'}"
                            moved = True
                        elif key == 'x':
                            self.vx = 0.0 if self.vx == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Back {'ON' if self.vx else 'OFF'}"
                            moved = True
                        elif key == 'a':
                            self.vy = 0.0 if self.vy == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Left {'ON' if self.vy else 'OFF'}"
                            moved = True
                        elif key == 'd':
                            self.vy = 0.0 if self.vy == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Right {'ON' if self.vy else 'OFF'}"
                            moved = True
                        elif key == 'q':
                            self.wz = 0.0 if self.wz == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Yaw L {'ON' if self.wz else 'OFF'}"
                            moved = True
                        elif key == 'e':
                            self.wz = 0.0 if self.wz == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Yaw R {'ON' if self.wz else 'OFF'}"
                            moved = True
                        elif key == 's':
                            self.vx = self.vy = self.wz = 0.0
                            self.last_action = 'Move Stop'
                            moved = True

                        if moved:
                            self.motion = 'Go' if (self.vx or self.vy or self.wz) else 'Stop'
                        elif key == 'i':
                            self.pitch += self.rpy_step
                            self.last_action = f'Pitch: {self.pitch:.2f}'
                        elif key == 'k':
                            self.pitch -= self.rpy_step
                            self.last_action = f'Pitch: {self.pitch:.2f}'
                        elif key == 'j':
                            self.roll += self.rpy_step
                            self.last_action = f'Roll: {self.roll:.2f}'
                        elif key == 'l':
                            self.roll -= self.rpy_step
                            self.last_action = f'Roll: {self.roll:.2f}'
                        elif key == 'u':
                            self.yaw += self.rpy_step
                            self.last_action = f'Yaw: {self.yaw:.2f}'
                        elif key == 'o':
                            self.yaw -= self.rpy_step
                            self.last_action = f'Yaw: {self.yaw:.2f}'
                        elif key == 'r':
                            self.z = min(1.0, self.z + 0.1)
                            self.last_action = f'Height: {self.z:.2f}'
                        elif key == 'f':
                            self.z = max(-1.0, self.z - 0.1)
                            self.last_action = f'Height: {self.z:.2f}'
                        elif key == 'b':
                            self.roll = self.pitch = self.yaw = self.z = 0.0
                            self.last_action = 'Pose Reset'
                        elif key in (']', '}'):
                            self.scale = min(1.0, self.scale + 0.1)
                            if self.vx > 0:
                                self.vx = 1.0 * self.scale
                            elif self.vx < 0:
                                self.vx = -1.0 * self.scale
                            self.last_action = f'Scale: {self.scale:.2f}'
                        elif key in ('[', '{'):
                            self.scale = max(0.01, self.scale - 0.1)
                            if self.vx > 0:
                                self.vx = 1.0 * self.scale
                            elif self.vx < 0:
                                self.vx = -1.0 * self.scale
                            self.last_action = f'Scale: {self.scale:.2f}'
                        elif key in ('=', '+'):
                            self.send_pulse(0.5)
                        elif key == '-':
                            self.send_pulse(-0.5)
                        elif key == 'm':
                            self.imu_auto_pose = not self.imu_auto_pose
                            self.last_action = f"IMU: {'ON' if self.imu_auto_pose else 'OFF'}"
                        elif key == 'v':
                            self.movement = 'Viewing' if self.movement == 'Stepping' else 'Stepping'
                            self.last_action = f'Mode -> {self.movement}'
                        elif key == '3':
                            self.movement = 'Viewing'
                            self.pose_cmd = 'Sit'
                            self.motion = 'Stop'
                            self.last_action = 'Sit Pose'
                        elif key == 'c':
                            self.movement = 'Viewing'
                            self.pose_cmd = 'Zero'
                            self.motion = 'Stop'
                            self.last_action = 'Zero Pose'
                        elif key == '5':
                            self.movement = 'Viewing'
                            self.pose_cmd = '0deg'
                            self.motion = '0deg'
                            self.last_action = '0deg Pose'
                        elif key == 'p':
                            self.motion = 'Go'
                            self.pose_cmd = 'Normal'
                            self.last_action = 'Motion GO'
                        elif key == ' ':
                            self.motion = 'Stop'
                            self.pose_cmd = 'Normal'
                            self.vx = self.vy = self.wz = 0.0
                            self.last_action = 'TOTAL STOP'

                    mini_cmd = MiniCmd()
                    mini_cmd.x_velocity = float(self.vx)
                    mini_cmd.y_velocity = float(self.vy)
                    mini_cmd.rate = float(self.wz)
                    mini_cmd.roll = float(self.roll)
                    mini_cmd.pitch = float(self.pitch)
                    mini_cmd.yaw = float(self.yaw)
                    mini_cmd.z = float(self.z)
                    mini_cmd.motion = self.motion
                    mini_cmd.movement = self.movement
                    mini_cmd.pose_cmd = self.pose_cmd
                    mini_cmd.imu_auto_pose = self.imu_auto_pose

                    joy_buttons = JoyButtons()
                    if key == 't':
                        joy_buttons.updown = 1
                        self.last_action = 'Clearance Up'
                    elif key == 'g':
                        joy_buttons.updown = -1
                        self.last_action = 'Clearance Down'

                    self.cmd_pub.publish(mini_cmd)
                    self.jb_pub.publish(joy_buttons)

                self.display_menu(force=bool(key))
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = SpotKeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    try:
        node.run()
    except Exception:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
