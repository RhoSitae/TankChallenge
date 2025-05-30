#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from functools import partial
import requests

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 내부 상태 초기값
        self.move_ws   = 'W'
        self.ws_weight = 0.0
        self.move_ad   = 'A'
        self.ad_weight = 0.0
        self.turret_qe = 'Q'
        self.qe_weight = 0.0
        self.turret_rf = 'R'
        self.rf_weight = 0.0

        # API URL
        self.api_url = "http://192.168.0.200:5000/update_command"

        # 토픽 콜백
        self.create_subscription(
            String, '/speed_command',
            partial(self.movews_callback, 'moveWS'),
            qos
        )
        self.create_subscription(
            String, '/steering_command',
            partial(self.movead_callback, 'moveAD'),
            qos
        )

        self.create_subscription(
            String, '/turret_yaw_control',
            partial(self.turretqe_callback, 'turretQE'),
            qos
        )

        self.create_subscription(
            String, '/turret_pitch_control',
            partial(self.turretrf_callback, 'turretRF'),
            qos
        )

        # 최초 구성
        self._rebuild()

        self.get_logger().info('CommandNode started')

    def movews_callback(self, topic_key: str, msg: String):
        """ 속도/조향 토픽이 들어올 때마다 내부 변수 갱신 → 전체 payload 재구성 → 즉시 전송 """
        try:
            direction, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().error(f"Invalid format on {topic_key}: '{msg.data}'")
            return

        if topic_key == 'moveWS':
            self.move_ws   = direction
            self.ws_weight = weight

    def movead_callback(self, topic_key: str, msg: String):
        """ 속도/조향 토픽이 들어올 때마다 내부 변수 갱신 → 전체 payload 재구성 → 즉시 전송 """
        try:
            direction, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().error(f"Invalid format on {topic_key}: '{msg.data}'")
            return

        if topic_key == 'moveAD':
            self.move_ad   = direction
            self.ad_weight = weight

    def turretqe_callback(self, topic_key: str, msg: String):
        """ 속도/조향 토픽이 들어올 때마다 내부 변수 갱신 → 전체 payload 재구성 → 즉시 전송 """
        try:
            direction, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().error(f"Invalid format on {topic_key}: '{msg.data}'")
            return

        if topic_key == 'turretQE':
            self.turret_qe  = direction
            self.qe_weight = weight

    def turretrf_callback(self, topic_key: str, msg: String):
        """ 속도/조향 토픽이 들어올 때마다 내부 변수 갱신 → 전체 payload 재구성 → 즉시 전송 """
        try:
            direction, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().error(f"Invalid format on {topic_key}: '{msg.data}'")
            return

        if topic_key == 'turretRF':
            self.turret_rf  = direction
            self.rf_weight = weight

        # 전체 다시 만들기
        self._rebuild()

        # self.get_logger().info(f"[{topic_key}] ← {direction},{weight:.3f}")
        # 콜백 직후 바로 전송 (타이머 없이)
        self.send_command()

    def _rebuild(self):
        self.latest_command = {
            "moveWS":   {"command": self.move_ws, "weight": abs(self.ws_weight)},
            "moveAD":   {"command": self.move_ad, "weight": abs(self.ad_weight)},
            "turretQE": {"command": self.turret_qe, "weight": abs(self.qe_weight)},
            "turretRF": {"command": self.turret_rf,  "weight": abs(self.rf_weight)},
            "fire":     False
        }

    def send_command(self):
        #self.get_logger().info(f"Sending → {self.latest_command}")
        try:
            resp = requests.post(self.api_url,
                                 json=self.latest_command,
                                 timeout=2.0)
            if resp.status_code != 200:
                self.get_logger().warn(f"API returned: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
