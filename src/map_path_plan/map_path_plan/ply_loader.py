#!/usr/bin/env python3
"""ROS2 node to load a .ply point cloud, convert Unity→ROS coords, color by height, and publish as PointCloud2 for RViz2"""
import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tank_package.common.coord_utils import unity_point_to_ros

class PLYLoader(Node):
    def __init__(self):
        super().__init__('ply_loader')
        # 1) 파라미터로 PLY 파일 경로 선언
        ply_path = self.declare_parameter(
            'ply_path',
            'flat_and_hills_pcd(0.5).ply'
        ).value

        # 2) Open3D로 PLY 로드
        pcd = o3d.io.read_point_cloud(ply_path)
        raw_points = np.asarray(pcd.points, dtype=np.float32)  # Unity: (x, z, y)
        count = raw_points.shape[0]

        # 3) 높이(uy)값 범위 계산
        uy_vals = raw_points[:, 2]  # third column is Unity y (height)
        zmin = float(np.min(uy_vals))
        zmax = float(np.max(uy_vals))
        dz = zmax - zmin if zmax > zmin else 1.0

        # 4) PointField 정의: x,y,z + rgba
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # 5) PointCloud2 메시지 생성
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        cloud = PointCloud2(
            header=header,
            height=1,
            width=count,
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * count,
        )

        # 6) 데이터 패킹: Unity→ROS 변환 후 색상
        buf = bytearray()
        for ux, uz, uy in raw_points:
            # Unity 좌표 (x, z, y) to ROS
            ros_pt = unity_point_to_ros({'x': float(ux), 'y': float(uy), 'z': float(uz)})
            x = ros_pt.x
            y = ros_pt.y
            z = ros_pt.z
            # 높이 비율로 색상 결정 (높을수록 빨강)
            t = (uy - zmin) / dz
            r = int(255 * t)
            g = 0
            b = int(255 * (1 - t))
            a = 255
            rgba = (a << 24) | (r << 16) | (g << 8) | b
            buf += struct.pack('fffI', x, y, z, rgba)
        cloud.data = bytes(buf)

        # 7) Latched QoS 퍼블리셔 생성
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub_cloud = self.create_publisher(PointCloud2, '/ply_cloud', qos)

        # 8) 퍼블리시
        self.pub_cloud.publish(cloud)
        self.get_logger().info(f'Published colored PLY cloud with {count} points (transformed to ROS coords)')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = PLYLoader()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()