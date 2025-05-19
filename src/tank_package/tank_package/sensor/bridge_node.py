#!/usr/bin/env python3
"""ROS2 sensor-bridge node:
   • GET /get_info → TankState, EnemyState, LidarScan 토픽 발행
"""
import requests
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tank_package.common.logger import get_logger
from tank_package.common.qos import DEFAULT_QOS
from tank_package.common.coord_utils import unity_point_to_ros, unity_to_ros_euler
from tank_package_msg.msg import TankState, EnemyState, LidarScan, LidarPoint

_JSON_TIMEOUT = 2.0  # seconds

class SensorBridge(Node):
    def __init__(self) -> None:
        super().__init__("bridge_node")
        self.log = get_logger(self)

        # ─ Parameters ──────────────────────────────────────────────
        self.declare_parameter("server_ip", "192.168.0.2")
        self.declare_parameter("poll_hz",   10)
        self.declare_parameter("timeout",   _JSON_TIMEOUT)

        ip = self.get_parameter("server_ip").value
        self.url_info = f"http://{ip}:5050/get_info"

        # ─ Publishers ──────────────────────────────────────────────
        qos = DEFAULT_QOS
        self.pub_tank  = self.create_publisher(TankState,  "/tank_state",  qos)
        self.pub_enemy = self.create_publisher(EnemyState, "/enemy_state", qos)
        self.pub_scan  = self.create_publisher(LidarScan,  "/lidar_scan",  qos)

        # ─ Timer for JSON poll ─────────────────────────────────────
        hz = self.get_parameter("poll_hz").value
        self.create_timer(1.0/hz, self.poll_info, ReentrantCallbackGroup())
        self.log.info(f"Bridge Node up → polling {self.url_info}")

    def poll_info(self) -> None:
        try:
            r = requests.get(
                self.url_info,
                timeout=self.get_parameter("timeout").value
            )
            r.raise_for_status()
            data = r.json()
        except Exception as e:
            self.log.warn(f"/get_info failed: {e}")
            return

        try:
            self._publish_state(data)
        except Exception as e:
            self.log.error(f"Publish error: {e}")

    def _publish_state(self, d: dict) -> None:
        now = self.get_clock().now().to_msg()

        # TankState
        ts = TankState()
        ts.stamp    = now
        ts.position = unity_point_to_ros(d.get("playerPos", {}))

        # body_euler: Unity(Z, Y, X) -> ROS(roll, pitch, yaw)
        ts.body_euler.x, ts.body_euler.y, ts.body_euler.z = unity_to_ros_euler(
            d.get("playerBodyZ", 0.0),
            d.get("playerBodyY", 0.0),
            d.get("playerBodyX", 0.0)
        )

        # turret_euler: Unity(Z, Y, X) -> ROS(roll, pitch, yaw)
        tr, tp, ty = unity_to_ros_euler(
            d.get("playerTurretZ", 0.0),
            d.get("playerTurretY", 0.0),
            d.get("playerTurretX", 0.0)
        )
        ts.turret_euler.x = tr
        ts.turret_euler.y = -tp
        ts.turret_euler.z = ty

        ts.speed  = d.get("playerSpeed", 0.0)
        ts.health = d.get("playerHealth", 100.0)
        self.pub_tank.publish(ts)

        # EnemyState
        es = EnemyState()
        es.stamp    = now
        es.position = unity_point_to_ros(d.get("enemyPos", {}))

        es.body_euler.x, es.body_euler.y, es.body_euler.z = unity_to_ros_euler(
            d.get("enemyBodyZ", 0.0),
            d.get("enemyBodyY", 0.0),
            d.get("enemyBodyX", 0.0)
        )

        er, ep, ey = unity_to_ros_euler(
            d.get("enemyTurretZ", 0.0),
            d.get("enemyTurretY", 0.0),
            d.get("enemyTurretX", 0.0)
        )
        es.turret_euler.x = er
        es.turret_euler.y = -ep
        es.turret_euler.z = ey

        es.speed  = d.get("enemySpeed", 0.0)
        es.health = d.get("enemyHealth", 100.0)
        self.pub_enemy.publish(es)

        # LidarScan
        scan = LidarScan()
        scan.stamp  = now
        scan.origin = unity_point_to_ros(d.get("lidarOrigin", {}))
        for p in d.get("lidarPoints", []):
            lp = LidarPoint()
            lp.angle    = p.get("angle", 0.0)
            lp.distance = p.get("distance", 0.0)
            lp.position = unity_point_to_ros(p.get("position", {}))
            scan.points.append(lp)
        self.pub_scan.publish(scan)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = SensorBridge()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()
