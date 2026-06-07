#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Isaac Lab RL 정책을 실물 LUX/GIM6010-8 로 배포하는 ROS 2 노드.

입력:
  /cmd_vel         geometry_msgs/Twist
  /spot/imu        lux_msgs/IMUdata      (BNO085, 좌표계 보정 완료 전제)
  /motor_feedback  std_msgs/String JSON  (mit_publisher_ros2 피드백)

출력:
  /spot/joints      lux_msgs/JointAngles  (시각화/상태 확인용, degrees)
  /spot/joints_cal  lux_msgs/JointAngles  (캘리브레이션 적용, degrees)
"""

import copy
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lux.GaitGenerator.Bezier import BezierGait
from lux.Kinematics.SpotKinematics import SpotModel
from lux_msgs.msg import IMUdata, JointAngles
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray, String
import torch
import yaml

# ── 학습 환경(lux_env_cfg.py) 과 동기화된 상수 ──────────────────────────────
POLICY_HZ = 50.0
GAIT_PERIOD_S = 0.55              # lux_env_cfg.gait_period_s

# Bezier gait parameters (lux_env_cfg 와 동일)
STEPLENGTH_SCALE = 0.05           # lux_env_cfg.steplength_scale
YAW_SCALE = 1.25                  # lux_env_cfg.yaw_scale
BASE_STEP_VELOCITY = 0.001
CLEARANCE_HEIGHT = 0.04           # lux_env_cfg.clearance_height
PENETRATION_DEPTH = 0.003         # lux_env_cfg.penetration_depth
TSWING = 0.275                    # gait_period_s * swing_ratio = 0.55 * 0.5
RESIDUALS_SCALE = 0.02            # lux_env_cfg.residuals_scale

# Joint order: FL hip/upper/lower, FR, BL, BR (lux_cfg.LUX_JOINT_NAMES 순서)
# lux_cfg.LUX_DEFAULT_JOINT_POS (rad)
DEFAULT_JOINT_POS = np.array(
    [
        0.0, 0.75, -1.45,   # FL hip / upper / lower
        0.0, 0.75, -1.45,   # FR
        0.0, 0.85, -1.55,   # BL
        0.0, 0.85, -1.55,   # BR
    ],
    dtype=np.float32,
)

JOINT_POS_LIMIT_LO = np.array([-0.52, -0.09, -2.88] * 4, dtype=np.float32)
JOINT_POS_LIMIT_HI = np.array([0.52, 2.18, 0.09] * 4, dtype=np.float32)

VX_MIN = 0.0
VX_MAX = 1.2    # lux_env_cfg.command_vx_range = (0.0, 1.2)
VY_MAX = 0.15   # lux_env_cfg.command_vy_range = (-0.15, 0.15)
WZ_MAX = 0.3    # lux_env_cfg.command_yaw_rate_range = (-0.3, 0.3)


def _load_calibration(calib_yaml: str):
    direction = [-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]
    offset_deg = [0.0] * 12

    if not calib_yaml or not os.path.exists(calib_yaml):
        return direction, offset_deg, False

    with open(calib_yaml, 'r') as f:
        data = yaml.safe_load(f) or {}

    if '/**' in data:
        params = data['/**'].get('ros__parameters', {})
        calibration = params.get('calibration', {})
    else:
        calibration = data.get('calibration', data)

    direction = list(calibration.get('direction', direction))
    offset_deg = list(calibration.get('offset_deg', offset_deg))
    return direction, offset_deg, True


def _make_joint_msg(deg, step_or_view=False):
    msg = JointAngles()
    msg.fls = float(deg[0])
    msg.fle = float(deg[1])
    msg.flw = float(deg[2])
    msg.frs = float(deg[3])
    msg.fre = float(deg[4])
    msg.frw = float(deg[5])
    msg.bls = float(deg[6])
    msg.ble = float(deg[7])
    msg.blw = float(deg[8])
    msg.brs = float(deg[9])
    msg.bre = float(deg[10])
    msg.brw = float(deg[11])
    msg.step_or_view = bool(step_or_view)
    return msg


def _quat_rotate_inverse(qw, qx, qy, qz, v):
    q = np.array([qx, qy, qz], dtype=np.float32)
    v = np.array(v, dtype=np.float32)
    t = 2.0 * np.cross(q, v)
    return (v + qw * t + np.cross(q, t)).astype(np.float32)


def _clip(v, lo, hi):
    return np.minimum(np.maximum(v, lo), hi)


class LuxRLInterface(Node):
    """Isaac Lab TorchScript 정책을 ROS 2 하드웨어 토픽에 연결한다."""

    def __init__(self):
        super().__init__('lux_rl_interface')

        self.declare_parameter('policy_path', '')
        self.declare_parameter('calib_yaml', '')
        self.declare_parameter('cmd_vx_scale', 1.0)
        self.declare_parameter('cmd_vy_scale', 1.0)
        self.declare_parameter('cmd_wz_scale', 1.0)
        self.declare_parameter('use_direct_bno085', True)
        self.declare_parameter('bno085_rate_hz', 100.0)
        self.declare_parameter('idle_auto_pose', True)
        self.declare_parameter('idle_command_eps', 1e-3)
        self.declare_parameter('POSE_PID_Kp', 1.5)
        self.declare_parameter('POSE_PID_Ki', 0.1)
        self.declare_parameter('POSE_PID_Kd', 0.05)
        self.declare_parameter('POSE_PID_I_Limit', 0.5)
        self.declare_parameter('ROLL_OFFSET', -1.0)
        self.declare_parameter('PITCH_OFFSET', 5.8)
        self.declare_parameter('RPY_SCALE', 0.785)
        self.declare_parameter('Z_SCALE_CTRL', 0.15)
        self.declare_parameter('shoulder_length', 0.0905)
        self.declare_parameter('elbow_length', 0.210)
        self.declare_parameter('wrist_length', 0.210)
        self.declare_parameter('hip_x', 0.405)
        self.declare_parameter('hip_y', 0.12)
        self.declare_parameter('foot_x', 0.405)
        self.declare_parameter('foot_y', 0.301)
        self.declare_parameter('height', 0.30)
        self.declare_parameter('com_offset', -0.04)

        self.policy_path = str(self.get_parameter('policy_path').value)
        calib_yaml = str(self.get_parameter('calib_yaml').value)
        self.cmd_vx_scale = float(self.get_parameter('cmd_vx_scale').value)
        self.cmd_vy_scale = float(self.get_parameter('cmd_vy_scale').value)
        self.cmd_wz_scale = float(self.get_parameter('cmd_wz_scale').value)
        self.use_direct_bno085 = bool(self.get_parameter('use_direct_bno085').value)
        self.bno085_rate_hz = float(self.get_parameter('bno085_rate_hz').value)
        self.idle_auto_pose = bool(self.get_parameter('idle_auto_pose').value)
        self.idle_command_eps = float(self.get_parameter('idle_command_eps').value)
        self.pose_kp = float(self.get_parameter('POSE_PID_Kp').value)
        self.pose_ki = float(self.get_parameter('POSE_PID_Ki').value)
        self.pose_kd = float(self.get_parameter('POSE_PID_Kd').value)
        self.pose_i_limit = float(self.get_parameter('POSE_PID_I_Limit').value)
        self.roll_offset_deg = float(self.get_parameter('ROLL_OFFSET').value)
        self.pitch_offset_deg = float(self.get_parameter('PITCH_OFFSET').value)
        self.rpy_scale = float(self.get_parameter('RPY_SCALE').value)
        self.z_scale_ctrl = float(self.get_parameter('Z_SCALE_CTRL').value)

        if not calib_yaml:
            try:
                pkg_share = get_package_share_directory('lux')
                calib_yaml = os.path.join(pkg_share, 'config', 'gim_servo_calib.yaml')
            except Exception:
                calib_yaml = ''

        try:
            self.calib_dir, self.calib_offset, loaded = _load_calibration(calib_yaml)
            if loaded:
                self.get_logger().info(f'캘리브레이션 로드: {calib_yaml}')
            else:
                self.get_logger().warn('캘리브레이션 파일 없음. 기본값 사용.')
        except Exception as e:
            self.calib_dir = [-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]
            self.calib_offset = [0.0] * 12
            self.get_logger().warn(f'캘리브레이션 로드 실패: {e}')

        # 학습 환경 기준: obs=47, action=12
        self.policy = None
        self.policy_obs_dim = 47
        self.policy_action_dim = 12
        if self.policy_path and os.path.exists(self.policy_path):
            try:
                self.policy = torch.jit.load(self.policy_path, map_location='cpu')
                self.policy.eval()
                self.policy_obs_dim, self.policy_action_dim = self._infer_policy_dims()
                self.get_logger().info(f'정책 로드 완료: {self.policy_path}')
                self.get_logger().info(
                    f'정책 입출력 차원: obs={self.policy_obs_dim}, action={self.policy_action_dim}'
                )
            except Exception as e:
                self.get_logger().error(f'정책 로드 실패: {e}')
        else:
            self.get_logger().warn(
                '정책 파일 없음. 순수 Bezier 보행만 실행합니다. '
                'policy_path 파라미터를 확인하세요.'
            )

        self._phase = 0.0
        self._step_dt = 1.0 / POLICY_HZ
        self._prev_actions = np.zeros(12, dtype=np.float32)   # action_space = 12
        self._ang_vel_b = np.zeros(3, dtype=np.float32)
        self._proj_gravity_b = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._commands = np.zeros(3, dtype=np.float32)
        self._joint_pos_rad = DEFAULT_JOINT_POS.copy()
        self._joint_vel_rads = np.zeros(12, dtype=np.float32)
        self._feedback_ready = False
        self._hw_startup_done = False
        self._bno = None
        self._current_roll_deg = 0.0
        self._current_pitch_deg = 0.0
        self._current_roll_rate_degs = 0.0
        self._current_pitch_rate_degs = 0.0
        self._idle_integral_roll = 0.0
        self._idle_integral_pitch = 0.0

        self._spot = SpotModel(
            shoulder_length=float(self.get_parameter('shoulder_length').value),
            elbow_length=float(self.get_parameter('elbow_length').value),
            wrist_length=float(self.get_parameter('wrist_length').value),
            hip_x=float(self.get_parameter('hip_x').value),
            hip_y=float(self.get_parameter('hip_y').value),
            foot_x=float(self.get_parameter('foot_x').value),
            foot_y=float(self.get_parameter('foot_y').value),
            height=float(self.get_parameter('height').value),
            com_offset=float(self.get_parameter('com_offset').value),
        )

        # Bezier gait state
        self.T_bf0 = copy.deepcopy(self._spot.WorldToFoot)
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=self._step_dt, Tswing=TSWING)
        self._was_idle = True

        qos = QoSProfile(depth=1)
        self.create_subscription(IMUdata, 'spot/imu', self._cb_imu, qos)
        self.create_subscription(String, '/motor_feedback', self._cb_feedback, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmdvel, 10)

        self._pub_ja = self.create_publisher(JointAngles, '/spot/joints', qos)
        self._pub_ja_cal = self.create_publisher(JointAngles, '/spot/joints_cal', qos)
        self._pub_commands = self.create_publisher(Float32MultiArray, '/rl/commands', qos)
        self.create_timer(self._step_dt, self._policy_step)

        if self.use_direct_bno085:
            self._init_direct_bno085()

        self.get_logger().info(f'LUX RL interface ready at {POLICY_HZ:.0f} Hz')

    def _init_direct_bno085(self):
        try:
            import board
            import busio
            from adafruit_bno08x import (
                BNO_REPORT_ACCELEROMETER,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_ROTATION_VECTOR,
            )
            from adafruit_bno08x.i2c import BNO08X_I2C

            i2c = busio.I2C(board.SCL, board.SDA)
            self._bno = BNO08X_I2C(i2c)
            self._bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self._bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            period = 1.0 / max(self.bno085_rate_hz, 1.0)
            self.create_timer(period, self._read_direct_bno085)
            self.get_logger().info('BNO085 direct read enabled: accel, gyro, quaternion')
        except Exception as e:
            self._bno = None
            self.get_logger().warn(
                f'BNO085 direct read 초기화 실패. /spot/imu 구독값을 사용합니다: {e}'
            )

    def _read_direct_bno085(self):
        if self._bno is None:
            return
        try:
            gyro_x, gyro_y, gyro_z = self._bno.gyro
            accel_x, accel_y, accel_z = self._bno.acceleration
            quat_i, quat_j, quat_k, quat_real = self._bno.quaternion

            self._ang_vel_b = np.array([gyro_x, gyro_y, gyro_z], dtype=np.float32)
            roll_rad = math.atan2(
                2.0 * (quat_real * quat_i + quat_j * quat_k),
                1.0 - 2.0 * (quat_i * quat_i + quat_j * quat_j),
            )
            pitch_arg = 2.0 * (quat_real * quat_j - quat_k * quat_i)
            pitch_rad = math.asin(float(np.clip(pitch_arg, -1.0, 1.0)))
            self._current_roll_deg = math.degrees(roll_rad) - self.roll_offset_deg
            self._current_pitch_deg = math.degrees(pitch_rad) - self.pitch_offset_deg
            self._current_roll_rate_degs = math.degrees(float(gyro_x))
            self._current_pitch_rate_degs = math.degrees(float(gyro_y))
            self._proj_gravity_b = _quat_rotate_inverse(
                quat_real, quat_i, quat_j, quat_k,
                np.array([0.0, 0.0, -1.0], dtype=np.float32),
            )

            acc = np.array([accel_x, accel_y, accel_z], dtype=np.float32)
            norm = float(np.linalg.norm(self._proj_gravity_b))
            if norm > 1e-4:
                self._proj_gravity_b = (self._proj_gravity_b / norm).astype(np.float32)
            elif float(np.linalg.norm(acc)) > 1e-4:
                self._proj_gravity_b = (-acc / float(np.linalg.norm(acc))).astype(np.float32)
        except Exception as e:
            self.get_logger().warn(
                f'BNO085 direct read 실패: {e}',
                throttle_duration_sec=2.0,
            )

    def _infer_policy_dims(self):
        obs_dim = 47
        action_dim = 12
        for name, param in self.policy.named_parameters():
            shape = tuple(param.shape)
            if name.endswith('weight') and len(shape) == 2:
                obs_dim = int(shape[1])   # 첫 번째 레이어 입력 차원 = obs_dim
                break
        for name, param in self.policy.named_parameters():
            shape = tuple(param.shape)
            if name.endswith('weight') and len(shape) == 2:
                action_dim = int(shape[0])  # 마지막 레이어 출력 차원 = action_dim
        return obs_dim, action_dim

    def _cb_imu(self, msg: IMUdata):
        self._ang_vel_b = np.array(
            [
                math.radians(float(msg.gyro_x)),
                math.radians(float(msg.gyro_y)),
                math.radians(float(msg.gyro_z)),
            ],
            dtype=np.float32,
        )

        quat_norm = math.sqrt(
            msg.quat_w**2 + msg.quat_x**2 + msg.quat_y**2 + msg.quat_z**2
        )
        if quat_norm > 0.9:
            # 쿼터니언으로 roll/pitch 계산 및 중력 방향 산출
            roll_rad = math.atan2(
                2.0 * (msg.quat_w * msg.quat_x + msg.quat_y * msg.quat_z),
                1.0 - 2.0 * (msg.quat_x**2 + msg.quat_y**2),
            )
            sin_p = float(np.clip(
                2.0 * (msg.quat_w * msg.quat_y - msg.quat_z * msg.quat_x), -1.0, 1.0
            ))
            pitch_rad = math.asin(sin_p)
            real_roll  = math.degrees(roll_rad)  - self.roll_offset_deg
            real_pitch = math.degrees(pitch_rad) - self.pitch_offset_deg

            proj = _quat_rotate_inverse(
                msg.quat_w, msg.quat_x, msg.quat_y, msg.quat_z,
                np.array([0.0, 0.0, -1.0], dtype=np.float32),
            )
            norm = float(np.linalg.norm(proj))
            if norm > 1e-4:
                self._proj_gravity_b = (proj / norm).astype(np.float32)
        else:
            # 쿼터니언 없음 → Euler 필드 + 가속도 근사 fallback
            real_roll  = float(msg.roll)  - self.roll_offset_deg
            real_pitch = float(msg.pitch) - self.pitch_offset_deg

            acc = np.array([msg.acc_x, msg.acc_y, msg.acc_z], dtype=np.float32)
            norm = float(np.linalg.norm(acc))
            if norm > 1e-4:
                self._proj_gravity_b = (-acc / norm).astype(np.float32)
            else:
                roll  = math.radians(real_roll)
                pitch = math.radians(real_pitch)
                self._proj_gravity_b = np.array(
                    [
                        math.sin(pitch),
                        -math.sin(roll) * math.cos(pitch),
                        -math.cos(roll) * math.cos(pitch),
                    ],
                    dtype=np.float32,
                )

        self._current_roll_deg        = real_roll
        self._current_pitch_deg       = real_pitch
        self._current_roll_rate_degs  = float(msg.gyro_x)
        self._current_pitch_rate_degs = float(msg.gyro_y)

    def _cb_feedback(self, msg: String):
        """mit_publisher /motor_feedback JSON 파싱.

        joint_pos_rad / joint_vel_radps 필드(IK 좌표계, rad)를 우선 사용한다.
        구버전 피드백에는 encoders(deg) / encoder_vels(rad/s) 로 fallback.
        """
        try:
            data = json.loads(msg.data)

            joint_pos_rad = data.get('joint_pos_rad')
            if joint_pos_rad and len(joint_pos_rad) >= 12:
                for i in range(12):
                    self._joint_pos_rad[i] = float(joint_pos_rad[i])
            else:
                encoders = data.get('encoders', [0.0] * 12)
                for i in range(12):
                    self._joint_pos_rad[i] = math.radians(float(encoders[i]))

            joint_vel_radps = data.get('joint_vel_radps')
            if joint_vel_radps and len(joint_vel_radps) >= 12:
                for i in range(12):
                    self._joint_vel_rads[i] = float(joint_vel_radps[i])
            else:
                encoder_vels = data.get('encoder_vels', [0.0] * 12)
                for i in range(12):
                    self._joint_vel_rads[i] = float(encoder_vels[i])

            self._feedback_ready = True
            if not self._hw_startup_done:
                if data.get('state_mode', 'STARTUP(Slow)') == 'NORMAL':
                    self._hw_startup_done = True
                    self.get_logger().info('mit_publisher startup 완료 → RL 정책 시작')
        except Exception as e:
            self.get_logger().warn(
                f'feedback 파싱 실패: {e}',
                throttle_duration_sec=2.0,
            )

    def _cb_cmdvel(self, msg: Twist):
        self._commands[0] = np.clip(float(msg.linear.x) * self.cmd_vx_scale, VX_MIN, VX_MAX)
        self._commands[1] = np.clip(float(msg.linear.y) * self.cmd_vy_scale, -VY_MAX, VY_MAX)
        self._commands[2] = np.clip(float(msg.angular.z) * self.cmd_wz_scale, -WZ_MAX, WZ_MAX)
        out = Float32MultiArray()
        out.data = [float(self._commands[0]), float(self._commands[1]), float(self._commands[2])]
        self._pub_commands.publish(out)

    def _policy_step(self):
        self._phase = math.fmod(self._phase + self._step_dt / GAIT_PERIOD_S, 1.0)

        if not self._hw_startup_done:
            self._publish_default_pose()
            return

        is_idle = self.idle_auto_pose and not self._has_motion_command()
        if is_idle:
            self._was_idle = True
            self._publish_idle_auto_pose()
            return

        # 아이들→보행 전환 시 BezierGait 상태 초기화로 급격한 발 움직임 방지
        if self._was_idle:
            self.T_bf = copy.deepcopy(self.T_bf0)
            self._was_idle = False

        self._idle_integral_roll = 0.0
        self._idle_integral_pitch = 0.0

        if self.policy is None:
            raw_actions = np.zeros(12, dtype=np.float32)
        else:
            obs = self._build_observation()
            with torch.no_grad():
                out = self.policy(torch.from_numpy(obs).unsqueeze(0))
            raw_actions = out.squeeze(0).cpu().numpy().astype(np.float32)

        actions = np.clip(raw_actions[:12], -1.0, 1.0)
        self._prev_actions = actions.copy()

        # Bezier 보행 궤적 계산
        vx = float(self._commands[0])
        vy = float(self._commands[1])
        wz = float(self._commands[2])

        step_length = float(np.clip(vx + abs(vy * 0.66), -1.0, 1.0)) * STEPLENGTH_SCALE
        lateral_fraction = vy * math.pi / 2.0
        yaw_rate = wz * YAW_SCALE

        self.T_bf = self.bzg.GenerateTrajectory(
            step_length, lateral_fraction, yaw_rate, BASE_STEP_VELOCITY,
            self.T_bf0, self.T_bf, CLEARANCE_HEIGHT, PENETRATION_DEPTH,
            [0, 0, 0, 0], self._step_dt,
        )

        # RL Cartesian 잔차(4발 × 3축) 를 발 위치에 가산
        T_bf_copy = copy.deepcopy(self.T_bf)
        foot_delta = actions.reshape(4, 3) * RESIDUALS_SCALE
        T_bf_copy['FL'][:3, 3] += foot_delta[0]
        T_bf_copy['FR'][:3, 3] += foot_delta[1]
        T_bf_copy['BL'][:3, 3] += foot_delta[2]
        T_bf_copy['BR'][:3, 3] += foot_delta[3]

        # 해석적 IK → 관절각(라디안)
        try:
            joint_angles = self._spot.IK(np.zeros(3), np.zeros(3), T_bf_copy)
        except Exception as e:
            self.get_logger().warn(
                f'IK 실패: {e}',
                throttle_duration_sec=2.0,
            )
            return

        target_rad = np.array(
            [joint_angles[r][c] for r in range(4) for c in range(3)],
            dtype=np.float32,
        )
        target_rad = np.clip(target_rad, JOINT_POS_LIMIT_LO, JOINT_POS_LIMIT_HI)
        target_deg = np.degrees(target_rad)

        cal_deg = [
            self.calib_dir[i] * (float(target_deg[i]) + self.calib_offset[i])
            for i in range(12)
        ]

        self._pub_ja.publish(_make_joint_msg(target_deg, step_or_view=False))
        self._pub_ja_cal.publish(_make_joint_msg(cal_deg, step_or_view=False))

    def _publish_default_pose(self):
        target_deg = np.degrees(DEFAULT_JOINT_POS)
        cal_deg = [
            self.calib_dir[i] * (float(target_deg[i]) + self.calib_offset[i])
            for i in range(12)
        ]
        self._pub_ja.publish(_make_joint_msg(target_deg, step_or_view=True))
        self._pub_ja_cal.publish(_make_joint_msg(cal_deg, step_or_view=True))

    def _has_motion_command(self) -> bool:
        return bool(np.linalg.norm(self._commands) > self.idle_command_eps)

    def _publish_idle_auto_pose(self):
        target_deg = self._compute_idle_auto_pose_deg()
        if target_deg is None:
            return

        cal_deg = [
            self.calib_dir[i] * (float(target_deg[i]) + self.calib_offset[i])
            for i in range(12)
        ]
        self._pub_ja.publish(_make_joint_msg(target_deg, step_or_view=True))
        self._pub_ja_cal.publish(_make_joint_msg(cal_deg, step_or_view=True))

    def _compute_idle_auto_pose_deg(self):
        target_roll_deg = 0.0
        target_pitch_deg = 0.0
        target_yaw_rad = 0.0
        pos = np.array([0.0, 0.0, 0.0 * self.z_scale_ctrl], dtype=np.float64)

        error_roll = target_roll_deg - self._current_roll_deg
        error_pitch = target_pitch_deg - self._current_pitch_deg

        self._idle_integral_roll = _clip(
            self._idle_integral_roll + error_roll * self._step_dt,
            -self.pose_i_limit,
            self.pose_i_limit,
        )
        self._idle_integral_pitch = _clip(
            self._idle_integral_pitch + error_pitch * self._step_dt,
            -self.pose_i_limit,
            self.pose_i_limit,
        )

        derivative_roll = -self._current_roll_rate_degs
        derivative_pitch = -self._current_pitch_rate_degs

        orn_roll_rad = -math.radians(
            (self.pose_kp * error_roll) +
            (self.pose_ki * self._idle_integral_roll) +
            (self.pose_kd * derivative_roll)
        )
        orn_pitch_rad = -math.radians(
            (self.pose_kp * error_pitch) +
            (self.pose_ki * self._idle_integral_pitch) +
            (self.pose_kd * derivative_pitch)
        )
        orn = np.array([orn_roll_rad, orn_pitch_rad, target_yaw_rad], dtype=np.float64)

        try:
            joint_angles = self._spot.IK(orn, pos, self.T_bf0)
        except Exception as e:
            self.get_logger().warn(
                f'idle IMU auto pose IK 실패: {e}',
                throttle_duration_sec=2.0,
            )
            return None

        target_rad = np.array(
            [joint_angles[r][c] for r in range(4) for c in range(3)],
            dtype=np.float32,
        )
        target_rad = np.clip(target_rad, JOINT_POS_LIMIT_LO, JOINT_POS_LIMIT_HI)
        return np.degrees(target_rad).astype(np.float32)

    def _build_observation(self) -> np.ndarray:
        """학습 환경 lux_env.py _get_observations() 와 동일한 47차원 관측 벡터 구성.

        [ang_vel_b(3), proj_gravity_b(3), commands(3),
         joint_pos_rel(12), joint_vel(12), prev_actions(12), phase(2)]
        """
        sin_phase = math.sin(2.0 * math.pi * self._phase)
        cos_phase = math.cos(2.0 * math.pi * self._phase)
        joint_pos_rel = self._joint_pos_rad - DEFAULT_JOINT_POS
        obs = np.concatenate([
            self._ang_vel_b,
            self._proj_gravity_b,
            self._commands,
            joint_pos_rel,
            self._joint_vel_rads,
            self._prev_actions,
            np.array([sin_phase, cos_phase], dtype=np.float32),
        ])
        if obs.shape[0] > self.policy_obs_dim:
            obs = obs[:self.policy_obs_dim]
        elif obs.shape[0] < self.policy_obs_dim:
            obs = np.pad(obs, (0, self.policy_obs_dim - obs.shape[0]))
        return obs.astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LuxRLInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
