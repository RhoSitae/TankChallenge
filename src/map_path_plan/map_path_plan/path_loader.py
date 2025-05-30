#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from scipy import ndimage
from scipy.ndimage import binary_erosion, binary_dilation
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Point, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tank_package.common.coord_utils import (
    unity_point_to_ros,
    ros_point_to_unity,
    ros_to_unity_euler,
)
from tank_package_msg.msg import TankState, EnemyState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

# A* on 2D grid
def astar_2d(occ2d, start, goal):
    neigh = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]
    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_set = [(h(start, goal), 0, start, None)]
    came, gscore = {}, {start: 0}
    while open_set:
        f, g, cur, parent = heapq.heappop(open_set)
        if cur in came:
            continue
        came[cur] = parent
        if cur == goal:
            path, n = [], cur
            while n:
                path.append(n)
                n = came[n]
            return path[::-1]
        for dx, dy in neigh:
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < occ2d.shape[0] and 0 <= nb[1] < occ2d.shape[1]):
                continue
            if occ2d[nb]:
                continue
            ng = g + 1
            if ng < gscore.get(nb, float('inf')):
                gscore[nb] = ng
                heapq.heappush(open_set, (ng + h(nb, goal), ng, nb, cur))
    return []

class OGMAndPathLoader(Node):
    def __init__(self):
        super().__init__('ogm_and_path_loader')

        # 시작점의 ROS 좌표 (x,y,z)
        self.start_ros = None
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # parameters
        npz_path = self.declare_parameter('npz_path', 'flat_and_hills_whole_OGM(0.5)_with_meta.npz').value
        slope_deg = float(self.declare_parameter('slope_threshold_deg', 30.0).value)
        ker_e = int(self.declare_parameter('erosion_kernel', 3).value)
        ker_d = int(self.declare_parameter('dilation_kernel', 8).value)
        self.declare_parameter('fov_range', 100.0)
        # 목표점 최소/최대 반경 (m)
        self.declare_parameter('min_goal_dist', 5.0)
        self.declare_parameter('max_goal_dist', 200.0)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.max_goal_dist = float(self.get_parameter('max_goal_dist').value)
        
        # load and build 2D occupancy
        meta = np.load(npz_path)
        self.ogm = meta['data']
        self.unity_o = meta['origin']
        self.res = float(meta['resolution'])
        occ3d = self.ogm.astype(bool)
        idx_max = np.argmax(occ3d, axis=2)
        H = idx_max * self.res
        Hmin = np.nanmin(H)
        Hf = np.where(np.isnan(H), Hmin, H)
        gx = ndimage.sobel(Hf, axis=0, mode='nearest') / (8 * self.res)
        gz = ndimage.sobel(Hf, axis=1, mode='nearest') / (8 * self.res)
        slope = np.arctan(np.hypot(gx, gz))
        base = (slope > np.deg2rad(slope_deg)).astype(np.int8)
        base = binary_dilation(
            binary_erosion(base, structure=np.ones((ker_e, ker_e), bool)),
            structure=np.ones((ker_d, ker_d), bool)
        )
        self.occ2d_orig = base
        # prepare base OccupancyGrid for publishing
        self.base_msg = OccupancyGrid(
            header=Header(frame_id='map'),
            info=MapMetaData(
                map_load_time=Header().stamp,
                resolution=self.res,
                width=base.shape[0],
                height=base.shape[1],
                origin=Pose()
            ),
            data=(base.T.flatten() * 100).astype(np.int8).tolist()
        )
        # state
        self.start_idx = None
        self.goal_idx = None
        self.enemy_pt = None
        self.enemy_yaw = None
        self.path_pts = None
        self.need_replan = True
        self.max_dist = 5.0
        
        self.fov_range = float(self.get_parameter('fov_range').value)
        # subscribers
        self.create_subscription(TankState, '/tank_state', self.cb_tank, qos)
        self.create_subscription(EnemyState, '/enemy_state', self.cb_enemy, qos)
        # publishers
        self.pub_masked = self.create_publisher(OccupancyGrid, '/occ2d_masked', qos)
        self.pub_points = self.create_publisher(Marker, '/occ2d_points', qos)
        self.pub_path = self.create_publisher(Marker, '/global_path', qos)
        # publish base once
        self.base_msg.header.stamp = self.get_clock().now().to_msg()

        # timer for continuous dynamic updates (FOV & path)
        self.create_timer(0.1, self.update_dynamic)  # 10 Hz dynamic update
        self.create_timer(10.0, self.auto_save_masked_unity_ogm)

    def auto_save_masked_unity_ogm(self):
        self.save_masked_unity_ogm()

    def save_masked_unity_ogm(self, filename='masked_unity_ogm_like_original.npz'):
        masked = self.mask_fov()
        if masked is None:
            self.get_logger().error("FOV 마스크가 None입니다.")
            return

        # 기존 포맷을 맞추기 위해 masked를 'data' 키로 저장
        np.savez(filename,
                 data=masked,  # ★ 이름을 'data'로
                 origin=self.unity_o,
                 resolution=self.res)
        self.get_logger().info(f"기존 ogm 포맷과 동일하게 '{filename}'에 저장되었습니다.")

    def cb_tank(self, msg: TankState):
        u = ros_point_to_unity({'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z})
        # 2D grid 인덱스
        idx = ((np.array([u.x, u.z, u.y]) - self.unity_o) / self.res).astype(int)
        self.start_idx = tuple(idx[:2])
        # 3D ROS 좌표로 저장 (replan 체크용)
        ros_start = unity_point_to_ros({'x': u.x, 'z': u.z, 'y': u.y})
        self.start_ros = np.array([ros_start.x, ros_start.y, ros_start.z])
        

    def cb_enemy(self, msg: EnemyState):
        u = ros_point_to_unity({'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z})
        self.enemy_pt = u
        self.enemy_yaw = ros_to_unity_euler(0, 0, msg.body_euler.z)[2]
        idx = ((np.array([u.x, u.z, u.y]) - self.unity_o) / self.res).astype(int)
        self.goal_idx = tuple(idx[:2])
        self.update_dynamic()

    def mask_fov(self):
        m = self.occ2d_orig.copy()
        if self.enemy_pt is None or self.enemy_yaw is None:
            return m

        # 1) 도(°) → 라디안 변환
        yaw_rad   = np.deg2rad(self.enemy_yaw)
        half_rad  = np.deg2rad(30)   # ±60°

        # 2) 라디안 각도로 ray 샘플링
        angles = np.linspace(yaw_rad + half_rad , yaw_rad - half_rad + 3.14, 600)
        map_max_r = max(m.shape) * self.res
        max_r = min(map_max_r, self.fov_range)
        ex, ez = self.enemy_pt.x, self.enemy_pt.z

        for ang in angles:
            r = 0.0
            while r < max_r:
                ux = ex + r * np.cos(ang)
                uz = ez + r * np.sin(ang)
                i  = int((ux - self.unity_o[0]) / self.res)
                j  = int((uz - self.unity_o[1]) / self.res)
                if not (0 <= i < m.shape[0] and 0 <= j < m.shape[1]):
                    break
                m[i, j] = 1
                if self.occ2d_orig[i, j] == 1:
                    break
                r += self.res

        # 3) 디버그: 도(°) 단위로 출력
        start_deg = self.enemy_yaw - 60
        end_deg   = self.enemy_yaw + 60
        self.get_logger().warn(f"[FOV] {start_deg:.1f}° → {end_deg:.1f}° (yaw={self.enemy_yaw:.1f}°)")

        return m


    def update_dynamic(self):
        masked = self.mask_fov()
        # publish masked occupancy
        grid = OccupancyGrid(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            info=self.base_msg.info,
            data=(masked.T.flatten() * 100).astype(np.int8).tolist()
        )
        self.pub_masked.publish(grid)

        # publish masked points
        pts_marker = Marker(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            ns='occ2d_points', id=1, type=Marker.POINTS,
            scale=Vector3(x=self.res, y=self.res, z=0.0),
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        )
        for i, j in np.argwhere(masked):
            ux = self.unity_o[0] + (i + 0.5) * self.res
            uz = self.unity_o[1] + (j + 0.5) * self.res
            p = unity_point_to_ros({'x': ux, 'z': uz, 'y': self.unity_o[2]})
            pts_marker.points.append(Point(x=p.x, y=p.y, z=p.z))
        self.pub_points.publish(pts_marker)

        # plan and publish path
        # 1) start, enemy 위치 필요
        if self.start_idx is None or self.enemy_pt is None:
            return

        sx = self.unity_o[0] + self.start_idx[0] * self.res
        sz = self.unity_o[1] + self.start_idx[1] * self.res
        ex, ez = self.enemy_pt.x, self.enemy_pt.z

        # 3) 적→아군 방향 단위벡터
        dx, dz = sx - ex, sz - ez
        dist = np.hypot(dx, dz)
        if dist < 1e-3:
            self.get_logger().warn("아군과 적전차 위치가 너무 가깝습니다. 목표점 설정 불가")
            return

        # 4) 최소~최대 거리 범위 내에서 장애물 없는 첫 후보 탐색
        found = False
        for r in np.arange(self.min_goal_dist, self.max_goal_dist + self.res, self.res):
            cx = ex + dx/dist * r
            cz = ez + dz/dist * r
            ix = int((cx - self.unity_o[0]) / self.res)
            iz = int((cz - self.unity_o[1]) / self.res)
            # 맵 범위 체크
            if not (0 <= ix < self.occ2d_orig.shape[0] and 0 <= iz < self.occ2d_orig.shape[1]):
                continue
            # 장애물 없는 셀(0)인지 확인
            if masked[ix, iz] == 0:
                self.goal_idx = (ix, iz)
                found = True
                break
        if not found:
            self.get_logger().warn(
                f"{self.min_goal_dist}m~{self.max_goal_dist}m 범위 내 장애물 없는 목표점을 찾지 못했습니다."
            )
            return
        # -------------------------------------------------------

        if self.need_replan:
            path = astar_2d(masked, self.start_idx, self.goal_idx)
            self.get_logger().info(f"masked num : {np.count_nonzero(masked)}")
            if not path:
                self.get_logger().error("A* 경로 생성 실패: 생성실패")
                return
            ros_pts = []
            for x, z in path:
                zs = np.where(self.ogm[x, z, :])[0]
                k = np.max(zs) if zs.size else 0
                wx = self.unity_o[0] + x * self.res
                wz = self.unity_o[1] + z * self.res
                wy = self.unity_o[2] + (k + 0.5) * self.res
                p = unity_point_to_ros({'x': wx, 'z': wz, 'y': wy})
                ros_pts.append((p.x, p.y, p.z))
            self.path_pts = np.array(ros_pts)
            self.need_replan = False

        if self.path_pts is None or self.path_pts.size == 0:
            return
        path_marker = Marker(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            ns='global_path', id=0, type=Marker.LINE_STRIP,
            scale=Vector3(x=1.0, y=0.0, z=0.0),
            color=ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        )
        for x, y, z in self.path_pts:
            path_marker.points.append(Point(x=x, y=y, z=z))
        self.pub_path.publish(path_marker)

        # replan check: path_pts(N×3) – start_ros(3,) 허용
        if self.start_ros is not None and self.path_pts.size > 0:
            dist2 = np.min(np.linalg.norm(self.path_pts - self.start_ros, axis=1))
            if dist2 > self.max_dist:
                self.need_replan = True

def main(args=None):
    rclpy.init(args=args)
    node = OGMAndPathLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
