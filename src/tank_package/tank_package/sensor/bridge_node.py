#!/usr/bin/env python3
"""ROS2 sensor-bridge node:
   • GET /get_info → TankState, EnemyState, LidarScan 토픽 발행
"""
import requests
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from tank_package.common.logger import get_logger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tank_package.common.coord_utils import unity_point_to_ros, unity_to_ros_euler
from tank_package_msg.msg import TankState, EnemyState, LidarScan, LidarPoint

_JSON_TIMEOUT = 2.0  # seconds

class SensorBridge(Node):
    def __init__(self) -> None:
        super().__init__("bridge_node")
        self.log = get_logger(self)

        # ─ Parameters ──────────────────────────────────────────────
        self.declare_parameter("server_ip", "192.168.0.200")
        self.declare_parameter("poll_hz",   25)
        self.declare_parameter("timeout",   _JSON_TIMEOUT)

        ip = self.get_parameter("server_ip").value
        self.url_info = f"http://{ip}:5000/get_info"

        # ─ Publishers ─────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub_tank  = self.create_publisher(TankState,  "/tank_state",  qos)
        self.pub_enemy = self.create_publisher(EnemyState, "/enemy_state", qos)
        self.pub_scan  = self.create_publisher(LidarScan,  "/lidar_scan",  qos)

        # ─ Timer for JSON poll ────────────────────────────────────
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

        # TankState 발행
        ts = TankState()
        ts.stamp    = now
        ts.position = unity_point_to_ros(d.get("playerPos", {}))
        # JSON에서 playerSpeed 항목을 읽어 speed 필드에 할당
        ts.speed    = float(d.get("playerSpeed", 0.0))

        # body_euler: Unity(Z,Y,X) -> ROS(roll,pitch,yaw)
        bx, by, bz = (
            d.get("playerBodyX", 0.0),
            d.get("playerBodyY", 0.0),
            d.get("playerBodyZ", 0.0)
        )
        ts.body_euler.x, ts.body_euler.y, ts.body_euler.z = unity_to_ros_euler(bz, by, bx)

        # turret_euler: Unity(Z,Y,X) -> ROS(roll,pitch,yaw)
        tx, ty, tz = (
            d.get("playerTurretX", 0.0),
            d.get("playerTurretY", 0.0),
            d.get("playerTurretZ", 0.0)
        )
        tr, tp, tyaw = unity_to_ros_euler(tz, ty, tx)
        ts.turret_euler.x = tr
        ts.turret_euler.y = -tp
        ts.turret_euler.z = tyaw

        ts.health = float(d.get("playerHealth", 100.0))
        ts.distance = float(d.get("distance", 0.0))
        self.pub_tank.publish(ts)

        # EnemyState 발행
        es = EnemyState()
        es.stamp    = now
        es.position = unity_point_to_ros(d.get("enemyPos", {}))
        ex, ey, ez = (
            d.get("enemyBodyX", 0.0),
            d.get("enemyBodyY", 0.0),
            d.get("enemyBodyZ", 0.0)
        )
        es.body_euler.x, es.body_euler.y, es.body_euler.z = unity_to_ros_euler(ez, ey, ex)

        etx, ety, etz = (
            d.get("enemyTurretX", 0.0),
            d.get("enemyTurretY", 0.0),
            d.get("enemyTurretZ", 0.0)
        )
        er, ep, eyaw = unity_to_ros_euler(etz, ety, etx)
        es.turret_euler.x = er
        es.turret_euler.y = -ep
        es.turret_euler.z = eyaw

        es.speed  = float(d.get("enemySpeed", 0.0))
        es.health = float(d.get("enemyHealth", 100.0))
        self.pub_enemy.publish(es)

        # LidarScan 발행
        scan = LidarScan()
        scan.stamp  = now
        scan.origin = unity_point_to_ros(d.get("lidarOrigin", {}))
        for p in d.get("lidarPoints", []):
            lp = LidarPoint()
            lp.angle    = float(p.get("angle", 0.0))
            lp.distance = float(p.get("distance", 0.0))
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
