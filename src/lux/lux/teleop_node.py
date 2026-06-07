#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from lux_msgs.msg import JoyButtons

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info("STARTING NODE: Teleoperation (Python)")

        # 파라미터 선언 및 로드
        self.frequency = self.declare_parameter("frequency", 60.0).value
        
        # Axes 매핑
        self.axis_linear_x = self.declare_parameter("axis_linear_x", 1).value
        self.axis_linear_y = self.declare_parameter("axis_linear_y", 0).value
        self.axis_linear_z = self.declare_parameter("axis_linear_z", 3).value
        self.axis_angular = self.declare_parameter("axis_angular", 2).value
        
        # Scaling
        self.scale_linear = self.declare_parameter("scale_linear", 0.5).value
        self.scale_angular = self.declare_parameter("scale_angular", 0.5).value
        self.scale_bumper = self.declare_parameter("scale_bumper", 1.0).value

        # Buttons 매핑
        self.btn_switch = self.declare_parameter("button_switch", 0).value # B 버튼
        self.btn_estop = self.declare_parameter("button_estop", 1).value   # A 버튼
        self.btn_imu = self.declare_parameter("button_imu", 3).value       # Y 버튼
        
        # 기타 매핑 (필요에 따라 인덱스 조정)
        self.rb = self.declare_parameter("rb", 5).value
        self.lb = self.declare_parameter("lb", 4).value
        self.rt = self.declare_parameter("rt", 7).value
        self.lt = self.declare_parameter("lt", 6).value
        self.ud = self.declare_parameter("updown", 7).value # Axes 인덱스
        self.lr = self.declare_parameter("leftright", 6).value # Axes 인덱스

        self.debounce_thresh = self.declare_parameter("debounce_thresh", 0.2).value

        # 내부 상태 변수
        self.twist = Twist()
        self.estop_state = False
        self.last_estop_btn = 0
        self.last_switch_btn = 0
        self.last_time = self.get_clock().now()
        
        # IMU Auto 상태 변수
        self.imu_auto_state = False
        self.last_imu_btn = 0

        # JoyButtons 상태
        self.jb_msg = JoyButtons()

        # Service Client
        self.switch_mv_client = self.create_client(Empty, "switch_movement")

        # Publishers
        self.vel_pub = self.create_publisher(Twist, "teleop", 10)
        self.estop_pub = self.create_publisher(Bool, "estop", 10)
        self.jb_pub = self.create_publisher(JoyButtons, "joybuttons", 10)

        # IMU Auto 상태 Publisher
        self.imu_pub = self.create_publisher(Bool, "imu_auto", 10)

        # Subscriber
        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Timer
        self.create_timer(1.0 / self.frequency, self.timer_callback)

    def joy_callback(self, msg):
        # 1. E-Stop 토글 (A 버튼)
        try:
            curr_estop_btn = msg.buttons[self.btn_estop]
            if curr_estop_btn == 1 and self.last_estop_btn == 0:
                self.estop_state = not self.estop_state
                if self.estop_state:
                    self.get_logger().warn("E-STOP ACTIVATED!")
                else:
                    self.get_logger().info("E-STOP RELEASED.")
            self.last_estop_btn = curr_estop_btn

            # 2. 모드 전환 (B 버튼 -> 서비스 호출)
            curr_switch_btn = msg.buttons[self.btn_switch]
            if curr_switch_btn == 1 and self.last_switch_btn == 0:
                # 비동기 서비스 호출
                if self.switch_mv_client.service_is_ready():
                    req = Empty.Request()
                    self.switch_mv_client.call_async(req)
                    self.get_logger().info("Switch Movement Service Called")
            self.last_switch_btn = curr_switch_btn
            
            # 3. IMU Auto Pose 토글 (Y 버튼)
            curr_imu_btn = msg.buttons[self.btn_imu]
            if curr_imu_btn == 1 and self.last_imu_btn == 0:
                self.imu_auto_state = not self.imu_auto_state
                if self.imu_auto_state:
                    self.get_logger().info("IMU AUTO POSE : ON")
                else:
                    self.get_logger().info("IMU AUTO POSE : OFF")
            self.last_imu_btn = curr_imu_btn

            # 3. Twist 메시지 생성
            self.twist.linear.x = msg.axes[self.axis_linear_x] * self.scale_linear
            self.twist.linear.y = msg.axes[self.axis_linear_y] * self.scale_linear
            self.twist.linear.z = msg.axes[self.axis_linear_z] * self.scale_linear
            self.twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular
            
            # 4. JoyButtons 메시지 생성
            # Arrow Pad (Axes)
            if len(msg.axes) > max(self.ud, self.lr):
                self.jb_msg.updown = int(msg.axes[self.ud])
                self.jb_msg.leftright = int(-msg.axes[self.lr])
            
            # Bumpers (Buttons)
            if len(msg.buttons) > max(self.lt, self.rt):
                self.jb_msg.left_bump = int(msg.buttons[self.lt])
                self.jb_msg.right_bump = int(msg.buttons[self.rt])

        except IndexError:
            pass

    def timer_callback(self):
        # 주기적으로 상태 발행
        
        # E-Stop
        estop_msg = Bool()
        estop_msg.data = self.estop_state
        self.estop_pub.publish(estop_msg)

        # IMU Auto
        imu_msg = Bool()
        imu_msg.data = self.imu_auto_state
        self.imu_pub.publish(imu_msg)

        # Twist (E-Stop 걸리면 0 전송)
        if self.estop_state:
            self.vel_pub.publish(Twist())
        else:
            self.vel_pub.publish(self.twist)

        # JoyButtons
        self.jb_pub.publish(self.jb_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
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