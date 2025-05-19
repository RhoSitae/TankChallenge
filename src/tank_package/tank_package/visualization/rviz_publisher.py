#!/usr/bin/env python3
import rclpy
import struct
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from tank_package_msg.msg import TankState, LidarScan
from tf_transformations import quaternion_from_euler, quaternion_multiply

class RvizPub(Node):
    def __init__(self):
        super().__init__("rviz_publisher")
        qos = QoSProfile(depth=10)

        # Publishers
        self.pub_tank   = self.create_publisher(Marker,      "/tank_marker",   qos)
        self.pub_axes   = self.create_publisher(Marker,      "/tank_axes",     qos)
        self.pub_turret = self.create_publisher(Marker,      "/turret_marker", qos)
        self.pub_lidar  = self.create_publisher(PointCloud2, "/lidar_points",  qos)

        # TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(TankState, "/tank_state", self.cb_tank,  qos)
        self.create_subscription(LidarScan, "/lidar_scan", self.cb_lidar, qos)

        self.get_logger().info("RvizPub node started, listening to /tank_state and /lidar_scan")

    def cb_tank(self, msg: TankState):
        # TF 전송
        t = TransformStamped()
        t.header.stamp    = msg.stamp
        t.header.frame_id = "map"
        t.child_frame_id  = "tank_base_link"
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        # 쿼터니언 변환
        q = quaternion_from_euler(
            math.radians(msg.body_euler.x),
            math.radians(msg.body_euler.y),
            math.radians(msg.body_euler.z)
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 1) Body orientation arrow
        roll  = math.radians(msg.body_euler.x)
        pitch = math.radians(msg.body_euler.y)
        yaw   = math.radians(msg.body_euler.z)
        base_q = quaternion_from_euler(roll, pitch, yaw)

        tank_marker = Marker(
            header=Header(frame_id="map", stamp=msg.stamp),
            ns="tank_body", id=0,
            type=Marker.ARROW,
            pose=Pose(
                position=msg.position,
                orientation=Quaternion(x=base_q[0], y=base_q[1], z=base_q[2], w=base_q[3])
            ),
            scale=Vector3(x=5.0, y=0.5, z=0.5),
            color=ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
        )
        self.pub_tank.publish(tank_marker)

        # 2) Body axes arrows
        rotations = [
            (0, base_q, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)),
            (1, quaternion_multiply(base_q, quaternion_from_euler(0, 0, math.pi/2)),
             ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)),
            (2, quaternion_multiply(base_q, quaternion_from_euler(0, -math.pi/2, 0)),
             ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)),
        ]
        for axis_id, q, col in rotations:
            axis_marker = Marker(
                header=Header(frame_id="map", stamp=msg.stamp),
                ns="tank_axes", id=axis_id,
                type=Marker.ARROW,
                pose=Pose(position=msg.position, orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])),
                scale=Vector3(x=5.0, y=0.5, z=0.5),
                color=col,
            )
            self.pub_axes.publish(axis_marker)

        # 3) Turret orientation arrow
        tr = math.radians(msg.turret_euler.x)
        tp = math.radians(msg.turret_euler.y)
        ty = math.radians(msg.turret_euler.z)
        turret_q = quaternion_from_euler(tr, tp, ty)
        turret_marker = Marker(
            header=Header(frame_id="map", stamp=msg.stamp),
            ns="tank_turret", id=10,
            type=Marker.ARROW,
            pose=Pose(position=msg.position, orientation=Quaternion(x=turret_q[0], y=turret_q[1], z=turret_q[2], w=turret_q[3])),
            scale=Vector3(x=5.0, y=0.5, z=0.5),
            color=ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        )
        self.pub_turret.publish(turret_marker)

    def cb_lidar(self, scan: LidarScan):
        pts = scan.points
        if not pts:
            return

        pc = PointCloud2(
            header=Header(frame_id="map", stamp=scan.stamp),
            height=1,
            width=len(pts),
            is_bigendian=False,
            is_dense=True,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=12 * len(pts),
        )
        buf = bytearray()
        for p in pts:
            buf += struct.pack('fff', p.position.x, p.position.y, p.position.z)
        pc.data = bytes(buf)
        self.pub_lidar.publish(pc)


def main():
    rclpy.init()
    node = RvizPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
