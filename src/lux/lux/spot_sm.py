#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from lux_msgs.msg import MiniCmd

# 위에서 만든 spot.py 모듈 임포트 (경로 주의)
from lux.spot import Spot

class SpotStateMachine(Node):
    def __init__(self):
        super().__init__('spot_sm')
        self.get_logger().info("STARTING NODE: spot_mini State Machine (Python)")

        # 파라미터
        self.declare_parameter("frequency", 200.0)
        frequency = self.get_parameter("frequency").value
        self.timeout = 1.0
        self.declare_parameter("decel_rate", 0.5)
        self.decel_rate = self.get_parameter("decel_rate").value
        self._dt = 1.0 / frequency

        # Spot 객체 생성
        self.spot_mini = Spot()

        # 내부 변수
        self.last_time = self.get_clock().now()
        self.motion_flag = False
        self.estop_engaged = False
        self.smoothed_vx   = 0.0
        self.smoothed_vy   = 0.0
        self.smoothed_rate = 0.0

        # Subscribers
        self.create_subscription(Twist, "teleop", self._cb_teleop, 10)
        self.create_subscription(Bool, "estop", self._cb_estop, 10)

        # Publisher
        self.mini_pub = self.create_publisher(MiniCmd, "mini_cmd", 10)

        # TeleopNode가 보내는 imu_auto 토픽 수신
        self.create_subscription(Bool, "imu_auto", self._cb_imu_auto, 10)

        # Service
        self.create_service(Empty, "switch_movement", self._cb_switch_movement)

        # Timer
        self.create_timer(1.0 / frequency, self._on_timer)

    def _ramp(self, current: float, target: float) -> float:
        """current를 target 방향으로 decel_rate 이하로 이동."""
        max_step = self.decel_rate * self._dt
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + math.copysign(max_step, diff)

    def _cb_teleop(self, msg):
        # 명령 업데이트 및 타임아웃 갱신
        self.spot_mini.update_command(
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.z, msg.angular.x, msg.angular.y
        )
        # [중요] 명령을 받으면 last_time을 갱신하여 타임아웃 방지
        self.last_time = self.get_clock().now()

    def _cb_estop(self, msg):
        if msg.data:
            # E-Stop 걸리면 모든 명령 0으로 초기화
            self.spot_mini.update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.motion_flag = True
            
            if not self.estop_engaged:
                self.get_logger().error("ENGAGING MANUAL E-STOP!")
                self.estop_engaged = True
            else:
                # 이미 걸려있는데 또 들어오면 Warn (상태 해제 아님에 주의)
                # 원본 C++ 코드 로직: estop->data가 True면 계속 경고/에러 상태 유지
                pass
        else:
            # E-Stop 해제 신호
            if self.estop_engaged:
                self.get_logger().warn("DIS-ENGAGING MANUAL E-STOP!")
                self.estop_engaged = False
        
        # E-Stop 신호도 통신이 살아있다는 증거로 볼 수 있음 (선택 사항)
        self.last_time = self.get_clock().now()

    def _cb_imu_auto(self, msg):
        self.spot_mini.update_imu_state(msg.data)
        # 통신이 살아있다는 증거이므로 타임아웃 갱신
        self.last_time = self.get_clock().now()

    def _cb_switch_movement(self, request, response):
        self.spot_mini.switch_movement()
        self.motion_flag = True
        return response

    def _on_timer(self):
        current_time = self.get_clock().now()
        cmd = self.spot_mini.return_command()
        mini_cmd = MiniCmd()

        # 타임아웃 체크
        time_diff = (current_time - self.last_time).nanoseconds / 1e9
        is_timeout = (time_diff > self.timeout)

        # E-Stop / timeout / 모드전환: 즉시 정지 (smoothed 값도 초기화)
        if self.estop_engaged or is_timeout or self.motion_flag:
            self.smoothed_vx   = 0.0
            self.smoothed_vy   = 0.0
            self.smoothed_rate = 0.0
            mini_cmd.x_velocity = 0.0
            mini_cmd.y_velocity = 0.0
            mini_cmd.rate       = 0.0
            mini_cmd.roll       = 0.0
            mini_cmd.pitch      = 0.0
            mini_cmd.yaw        = 0.0
            mini_cmd.z          = 0.0
            mini_cmd.faster     = 0.0
            mini_cmd.slower     = 0.0
            mini_cmd.motion     = "Stop"
            mini_cmd.movement   = cmd.movement
            mini_cmd.imu_auto_pose = cmd.imu_auto_pose
        else:
            # 정상 주행: velocity를 decel_rate로 선형화
            self.smoothed_vx   = self._ramp(self.smoothed_vx,   cmd.x_velocity)
            self.smoothed_vy   = self._ramp(self.smoothed_vy,   cmd.y_velocity)
            self.smoothed_rate = self._ramp(self.smoothed_rate, cmd.rate)

            near_zero = (
                abs(self.smoothed_vx)   < 0.01 and
                abs(self.smoothed_vy)   < 0.01 and
                abs(self.smoothed_rate) < 0.01
            )

            mini_cmd.x_velocity    = self.smoothed_vx
            mini_cmd.y_velocity    = self.smoothed_vy
            mini_cmd.rate          = self.smoothed_rate
            mini_cmd.roll          = cmd.roll
            mini_cmd.pitch         = cmd.pitch
            mini_cmd.yaw           = cmd.yaw
            mini_cmd.z             = cmd.z
            mini_cmd.faster        = cmd.faster
            mini_cmd.slower        = cmd.slower
            if cmd.movement == "Viewing":
                # Viewing 모드: roll/pitch/yaw/z 기반이므로 spot.py의 판정 사용
                mini_cmd.motion = cmd.motion
            else:
                mini_cmd.motion = "Stop" if near_zero else "Go"
            mini_cmd.movement      = cmd.movement
            mini_cmd.imu_auto_pose = cmd.imu_auto_pose

        if is_timeout:
            # 로그 스로틀링 (1초마다 출력)
            self.get_logger().error("TIMEOUT...ENGAGING E-STOP!", throttle_duration_sec=1.0)

        self.mini_pub.publish(mini_cmd)
        self.motion_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = SpotStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()