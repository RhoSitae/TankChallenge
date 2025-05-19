#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from tank_package_msg.msg import TankState

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 1) cmd_vel 퍼블리셔 (tank 시뮬레이터가 구독)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2) 외부(goal) 또는 자동 주행 알고리즘에서 넘어오는 목표
        self.sub_goal = self.create_subscription(
            Float32MultiArray,
            '/tank_control/goal',
            self.goal_callback,
            10
        )

        # 3) 센서 브리지에서 들어오는 현재 상태 구독
        self.sub_state = self.create_subscription(
            TankState,
            '/tank_state',
            self.state_callback,
            10
        )

        # 내부에 최신 상태를 저장
        self.current_state = None

        self.get_logger().info('ControlNode started')

    def state_callback(self, msg: TankState):
        # bridge_node.py가 보낸 ts = TankState
        # msg.position.x, .y, .z
        # msg.body_euler.x(pitch), .y(yaw), .z(roll)
        self.current_state = msg
        self.get_logger().info(
            f"[TankState] pos=({msg.position.x:.2f},"
            f"{msg.position.y:.2f},{msg.position.z:.2f}) "
            f"euler=(roll={msg.body_euler.x:.1f},"
            f"pitch={msg.body_euler.y:.1f},"
            f"yaw={msg.body_euler.z:.1f})"
        )

    def goal_callback(self, msg: Float32MultiArray):
        # 사용자가 직접 퍼블리시하거나, 자율 주행 로직의 출력
        # msg.data = [goal_x, goal_y, goal_yaw, goal_speed]
        goal_x, goal_y, goal_yaw, goal_speed = msg.data

        # 예시: 단순히 속도/회전량으로 매핑
        twist = Twist()
        twist.linear.x  = goal_speed
        twist.angular.z = goal_yaw
        self.pub_cmd.publish(twist)

        self.get_logger().info(f'→ cmd_vel: lin={twist.linear.x:.2f} '
                               f'ang={twist.angular.z:.2f}')

    def run(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = ControlNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
