#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from scipy import ndimage
from scipy.ndimage import binary_erosion, binary_dilation, binary_closing, binary_fill_holes
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


# 2D OGM에서 A*
def astar_2d(occ2d, start, goal):
    nx, ny = occ2d.shape
    neigh = [(1,0),(-1,0),(0,1),(0,-1), (1, 1), (-1, 1), (-1, -1), (1, -1)]
    # neigh = [(1,0),(-1,0),(0,1),(0,-1)]
    # neigh = [(2,0),(-2,0),(0,2),(0,-2), (2, 2), (-2, 2), (-2, -2), (2, -2)]
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start,goal), 0, start, None)]
    came, gscore = {}, {start:0}
    while open_set:
        f, g, cur, parent = heapq.heappop(open_set)
        if cur in came: continue
        came[cur] = parent
        if cur == goal:
            path=[]; n=cur
            while n: path.append(n); n=came[n]
            return path[::-1]
        for dx,dy in neigh:
            nb = (cur[0]+dx, cur[1]+dy)
            if not (0<=nb[0]<nx and 0<=nb[1]<ny): continue
            if occ2d[nb]: continue
            ng = g+1
            if ng < gscore.get(nb, float('inf')):
                gscore[nb] = ng
                heapq.heappush(open_set, (ng+h(nb,goal), ng, nb, cur))
    return []


def sample_ray_cells(start_pos: np.ndarray,          # ★ 월드 [x,z]
                    direction: np.ndarray,          # 단위 벡터
                     *,                             # 이후는 키워드 전용
                    origin: np.ndarray,
                    voxel_size: float,
                    map_shape: tuple[int,int],
                    max_range_m: float = 300.0,
                    return_mode: str = "index"):   # "world" | "index"
    """반직선을 따라 찍히는 셀 목록 반환."""
    n_steps = int(max_range_m / voxel_size)
    start_pos = np.array(start_pos) 
    cur     = start_pos.astype(float).copy()
    out_w, out_i = [], []

    for _ in range(n_steps):
        row, col = world_to_index(cur, origin, voxel_size)
        if not (0 <= row < map_shape[0] and 0 <= col < map_shape[1]):
            break
        out_w.append(cur.copy())        # 월드
        out_i.append((row, col))        # 인덱스
        cur += direction * voxel_size

    return out_w if return_mode=="world" else out_i

# heading 기준 시야각 +- 90도 벡터
def heading_to_unit_vec(theta_rad: float) -> np.ndarray:
    """2D 단위 벡터 [dx, dz] 반환 (x-z 평면)."""
    return np.array([np.cos(theta_rad), np.sin(theta_rad)])

# 수선의 발 내리는 함수
def foot_to_ray(P: np.ndarray,           # 내 전차 [x,z]
                O: np.ndarray,           # 적 전차 [x,z]
                v: np.ndarray,           # 반직선 방향 벡터
                *,                       # 이후는 **키워드 전용**
                origin: tuple,
                voxel_size: float,
                map_shape: tuple[int, int]
) -> tuple[np.ndarray, float, float]:
    """
    ▸ 수선의 발 F를 구하고, 맵 경계 안으로 클램핑.
    ▸ 반환: (F, d_self, d_enemy)
        · F         : 월드 좌표 (클램핑 결과)
        · d_self    : 내 전차 → F 거리
        · d_enemy   : 적 전차 → F 거리
    """
    # 1) 수선의 발 (무한 반직선 기준)
    u = v / np.linalg.norm(v)                 # 단위벡터
    t = max(np.dot(P - O, u), 0.0)            # O 기준 사영 길이
    F = O + t * u

    # 2) 맵 경계 클램핑 (ray 방향으로만 잘라냄)
    x_min = origin[0]
    z_min = origin[1]
    x_max = origin[0] + (map_shape[1] - 1) * voxel_size
    z_max = origin[1] + (map_shape[0] - 1) * voxel_size

    # 각 축별 허용 최대 t 계산
    if u[0] > 0:
        t_x = (x_max - O[0]) / u[0]
    elif u[0] < 0:
        t_x = (x_min - O[0]) / u[0]
    else:
        t_x = np.inf

    if u[1] > 0:
        t_z = (z_max - O[1]) / u[1]
    elif u[1] < 0:
        t_z = (z_min - O[1]) / u[1]
    else:
        t_z = np.inf

    t_max_rect = min(t_x, t_z)               # 지도 안에서 가능한 최대 t
    t_clamped  = np.clip(t, 0.0, t_max_rect)
    F          = O + t_clamped * u           # 클램핑된 수선의 발

    # 3) 거리 계산
    d_self  = np.linalg.norm(P - F)          # 내 전차 → F
    d_enemy = np.linalg.norm(O - F)          # 적 전차 → F
    return F, d_self, d_enemy

# 생성된 경유점 map 범위 고려하여 ray상에서 clamp
def clamp_point_on_ray(F: np.ndarray,        # 수선의 발(월드)
                       O: np.ndarray,        # 적 전차 위치
                       v: np.ndarray,        # ray 방향벡터
                       *,
                       origin: tuple,
                       voxel_size: float,
                       map_shape: tuple[int, int]) -> np.ndarray:
    """
    F 가 맵 밖이면, O→v 방향 선분을 지도 경계와 만나는 지점으로 잘라
    (= t 를 [0, t_max_rect] 로 제한) 반환합니다.
    """

    # 0) 단위벡터 & F 까지의 t
    u      = v / np.linalg.norm(v)
    t_F    = np.dot(F - O, u)          # O → F 거리(스칼라, m)

    # 1) ray 가 지도 밖으로 나갈 수 있는 최대 t 계산
    #    ─ x 경계
    if u[0] > 0:
        t_x = (origin[0] + (map_shape[1]-1)*voxel_size - O[0]) / u[0]
    elif u[0] < 0:
        t_x = (origin[0]                         - O[0]) / u[0]
    else:
        t_x = np.inf
    #    ─ z 경계
    if u[1] > 0:
        t_z = (origin[1] + (map_shape[0]-1)*voxel_size - O[1]) / u[1]
    elif u[1] < 0:
        t_z = (origin[1]                         - O[1]) / u[1]
    else:
        t_z = np.inf

    t_max_rect = min(t_x, t_z)           # 지도 안에서 허용되는 최대 t (≥0)

    # 2) 클램핑
    t_clamped  = np.clip(t_F, 0.0, t_max_rect)
    return O + t_clamped * u             # 월드 좌표

# world 좌표를 voxel단위 binary map 좌표로 변환
def world_to_index(world_xy: np.ndarray, origin: np.ndarray, voxel_size: float, *, round_mode="floor") -> tuple[int, int]:
    """월드 (x,z) → (row, col).  round_mode = 'floor' | 'nearest'"""
    world_xy = np.asarray(world_xy, dtype=float)       # ★ 추가
    origin   = np.asarray(origin[:2], dtype=float)     # ★ 추가
    rel = (world_xy - origin[:2]) / voxel_size
    idx = np.floor(rel).astype(int) if round_mode == "floor" else np.rint(rel).astype(int)
    return int(idx[0]), int(idx[1]) # 우 전방

# 경유점 생성 위해 내가 적 전차 시야각 기준 전/후방 구분 함수
def is_front(xs: float, zs: float,
            xg: float, zg: float,
            h_g: float) -> bool:
    """
    적 전차( xg, zg, heading h_g ) 기준으로
    내 전차( xs, zs )가 전방( True )인지 후방( False )인지 반환.
    ▷ h_g : math_rad (0 = +x축, CCW가 양의 방향)
    """
    # 1) 적 전차 헤딩 방향 단위벡터 u = [cos, sin]
    u = np.array([np.cos(h_g), np.sin(h_g)])   # (dx, dz)

    # 2) 상대 위치벡터 r = 내 위치 – 적 위치
    r = np.array([xs - xg, zs - zg])           # (Δx, Δz)

    # 3) 내적 부호로 전·후방 결정
    #    dot > 0  → 전방 (각도 < 90°)
    #    dot < 0  → 후방 (각도 > 90°)
    return np.dot(u, r) >= 0                   # 동일선상(90°)도 전방 처리

def unity_deg_to_math_rad(angle_deg):
    deg_math = 90.0 - angle_deg
    rad = np.deg2rad(deg_math)
    return (rad + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화

def find_free_on_ray(
    O_xy, u, start_t, *,                  # O : 적 전차 (x,z)
    binary_map, origin, voxel_size, map_shape,
    max_extra=300.0, step=0.5,
):
    """
    1)  O + t·u  선에서 0․5 m 간격으로 이동하며
    2)  먼저 적에게 **가까운 쪽(-t)** 으로 탐색,
        못 찾으면 **바깥(+t)** 으로 탐색.
    3)  binary_map==0 인 첫 셀을 (월드좌표, 인덱스) 로 반환.
        찾지 못하면 (None, None).
    """

    # ── 0. 단위벡터 정규화 ─────────────────────────────
    u = u / np.linalg.norm(u)

    # ── 1. 지도 경계까지 허용되는 t_max 계산 ──────────
    if u[0] > 0:
        t_x = (origin[0] + (map_shape[1]-1)*voxel_size - O_xy[0]) / u[0]
    elif u[0] < 0:
        t_x = (origin[0] - O_xy[0]) / u[0]
    else:
        t_x = np.inf

    if u[1] > 0:
        t_z = (origin[1] + (map_shape[0]-1)*voxel_size - O_xy[1]) / u[1]
    elif u[1] < 0:
        t_z = (origin[1] - O_xy[1]) / u[1]
    else:
        t_z = np.inf

    t_max = max(0.0, min(t_x, t_z))        # 0 ≤ t ≤ t_max

    # ── 2-A. “적에게 가까운 방향” 우선 탐색 (-t) ──────
    t = start_t
    n_steps = int(max_extra / step)
    for _ in range(n_steps):
        # 현재 t 검사
        P = O_xy + t * u
        row, col = world_to_index(P, origin, voxel_size)

        if (0 <= row < map_shape[0]) and (0 <= col < map_shape[1]):
            if binary_map[row, col] == 0:
                return P, (row, col)
        else:
            break                          # 맵을 벗어남

        t -= step                          # 적에게 더 접근
        if t < 0:                          # O 지점 지나면 중단
            break

    # ── 2-B. 못 찾았으면 바깥 방향(+t) 탐색 ───────────
    t = start_t + step                     # 이미 검사한 지점은 건너뜀
    for _ in range(n_steps):
        if t > t_max:
            break

        P = O_xy + t * u
        row, col = world_to_index(P, origin, voxel_size)

        if (0 <= row < map_shape[0]) and (0 <= col < map_shape[1]):
            if binary_map[row, col] == 0:
                return P, (row, col)
        else:
            break

        t += step                          # 바깥쪽으로 이동

    # ── 3. 실패 ───────────────────────────────────────
    return None, None

# def waypoint_on_approach_ring(O_xy,          # 적 전차 (x,z)
#                             heading,       # 적 전차 헤딩(rad)
#                             radius,        # approach_min
#                             binary_map, origin, voxel_size, map_shape,
#                             prefer_xy):    # 내 전차 (x,z)
#     """
#     ▸ 적 전방 ± 150° 방향으로 원(approach_min) 위 두 점을 계산
#     ▸ 맵 밖 / 장애물 셀은 제외
#     ▸ 남은 점 가운데 내 전차에 더 가까운 것을 반환
#     (없으면 None, None)
#     """
#     cand_pts = []
#     for sign in (+1, -1):                    # +150°, -150°
#         theta = heading + sign * math.radians(150)
#         pt    = O_xy + radius * np.array([math.cos(theta), math.sin(theta)])

#         # 맵 범위‧장애물 체크
#         row, col = world_to_index(pt, origin, voxel_size)   # (x,z) → (row,col)
#         if (0 <= row < map_shape[0] and 0 <= col < map_shape[1]
#                 and binary_map[row, col] == 0):
#             cand_pts.append((pt, (row, col)))
#     # print('링 점점점점', cand_pts)
#     if not cand_pts:
#         return None, None          # 둘 다 막혔거나 지도 밖
#     if len(cand_pts) == 1:
#         return cand_pts[0]         # 하나만 통과

#     # 두 점 다 가능 → 내 전차와 거리 비교
#     dists = [np.linalg.norm(prefer_xy - p[0]) for p,_ in cand_pts]
#     # print('경유점 호호호', prefer_xy)
#     # print('거ㄹ리리ㅣ릴', dists)
#     best  = cand_pts[int(np.argmin(dists))]
    # return best        
def waypoint_on_approach_ring(
    O_xy, heading, radius,
    binary_map, origin, voxel_size, map_shape,
    prefer_xy
):
    cand_pts = []
    for sign in (+1, -1):
        theta = heading + sign * math.radians(150)
        pt = O_xy + radius * np.array([math.cos(theta), math.sin(theta)])

        # 맵 밖 / 장애물 체크 전 클램핑
        pt_clamped = clamp_waypoint_on_ring(pt, O_xy, origin, voxel_size, map_shape)

        row, col = world_to_index(pt_clamped, origin, voxel_size)
        if (0 <= row < map_shape[0] and 0 <= col < map_shape[1]
                and binary_map[row, col] == 0):
            cand_pts.append((pt_clamped, (row, col)))

    if not cand_pts:
        return None, None
    if len(cand_pts) == 1:
        return cand_pts[0]

    dists = [np.linalg.norm(prefer_xy - p[0]) for p,_ in cand_pts]
    best = cand_pts[int(np.argmin(dists))]
    return best


def clamp_waypoint_on_ring(
    ring_pt: np.ndarray,         # ring 원 위 후보 점 (월드 좌표, 2D)
    center_pt: np.ndarray,       # 적 전차 위치 O_xy (월드 좌표, 2D)
    origin: tuple,
    voxel_size: float,
    map_shape: tuple[int, int]
) -> np.ndarray:
    """
    ring_pt가 맵 밖에 있으면 지도 경계와 만나는 점으로 클램핑,
    v는 center_pt → ring_pt 방향 벡터 (unit)
    """
    v = ring_pt - center_pt
    if np.linalg.norm(v) < 1e-6:
        return ring_pt  # 거의 같은 점이면 클램핑 불필요

    return clamp_point_on_ray(
        F=ring_pt,
        O=center_pt,
        v=v,
        origin=origin,
        voxel_size=voxel_size,
        map_shape=map_shape
    )


#######################################################################################################################
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
        ker_d2 = int(self.declare_parameter('dilation2_kernel', 7).value)
        self.declare_parameter('fov_range', 100.0)
        # 목표점 최소/최대 반경 (m)
        self.declare_parameter('min_goal_dist', 5.0)
        self.declare_parameter('max_goal_dist', 200.0)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.max_goal_dist = float(self.get_parameter('max_goal_dist').value)


        # load and build 2D occupancy
        meta = np.load(npz_path)
        self.ogm = meta['data'][:600, :600, :]
        self.nx, self.nz, self.ny = self.ogm.shape  
        self.unity_o = meta['origin']
        self.res = float(meta['resolution'])

        ################################### 3D to 2D OGM 

        occ3d = self.ogm.astype(bool)
        idx_max = np.argmax(occ3d, axis=2)
        H = idx_max * self.res

        ###### slope mask#########
        Hmin = np.nanmin(H)
        Hf = np.where(np.isnan(H), Hmin, H)
        gx = ndimage.sobel(Hf, axis=0, mode='nearest') / (8 * self.res)
        gz = ndimage.sobel(Hf, axis=1, mode='nearest') / (8 * self.res)
        slope = np.arctan(np.hypot(gx, gz))
        base = (slope > np.deg2rad(slope_deg)).astype(np.int8)
        ########################## base: 장애물 True False


        base = binary_dilation(
            binary_erosion(base, structure=np.ones((ker_e, ker_e), bool)),
            structure=np.ones((ker_e, ker_e), bool)
        )
        #self.occ2d_orig = base
        ####################################### 노이즈 제거(salt & pepper)

        ######## 전차 반경 고려하여 장애물 margin #######
        base = binary_dilation(base, structure=np.ones((ker_d, ker_d), bool))
        ########


        ######## 덕규 코드 구멍 채우기 #######
        base = binary_closing(base, structure=np.ones((ker_d2, ker_d2), bool)).astype(bool)
        base = binary_fill_holes(base).astype(bool)
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
        self.max_dist = 10.0
        self.approach_min = 50
        self.k0 = 0
        self.fov_range = float(self.get_parameter('fov_range').value)
        # subscribers
        self.create_subscription(TankState, '/tank_state', self.cb_tank, qos)
        self.create_subscription(EnemyState, '/enemy_state', self.cb_enemy, qos)
        # publishers
        self.pub_masked = self.create_publisher(OccupancyGrid, '/occ2d_masked', qos)
        self.pub_points = self.create_publisher(Marker, '/occ2d_points', qos)
        self.pub_path = self.create_publisher(Marker, '/global_path', qos)
        self.pub_wp_marker = self.create_publisher(Marker, '/wp_marker', qos)
        self.pub_wp_ring_marker = self.create_publisher(Marker, '/wp_ring_marker', qos)
        # publish base once
        self.base_msg.header.stamp = self.get_clock().now().to_msg()

        # timer for continuous dynamic updates (FOV & path)
        self.create_timer(0.1, self.update_dynamic)  # 10 Hz dynamic update
    #     self.create_timer(10.0, self.auto_save_masked_unity_ogm)

    # def auto_save_masked_unity_ogm(self):
    #     self.save_masked_unity_ogm()

    # def save_masked_unity_ogm(self, filename='masked_unity_ogm_like_original.npz'):
    #     masked = self.mask_fov()
    #     if masked is None:
    #         self.get_logger().error("FOV 마스크가 None입니다.")
    #         return

    #     # 기존 포맷을 맞추기 위해 masked를 'data' 키로 저장
    #     np.savez(filename,
    #              data=masked,  # ★ 이름을 'data'로
    #              origin=self.unity_o,
    #              resolution=self.res)
    #     self.get_logger().info(f"기존 ogm 포맷과 동일하게 '{filename}'에 저장되었습니다.")

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
        half_rad  = np.deg2rad(60)   # ±60°

        # 2) 라디안 각도로 ray 샘플링
        angles = np.linspace(yaw_rad + half_rad , yaw_rad - half_rad, 200)
        map_max_r = max(m.shape) * self.res
        max_r = min(map_max_r, self.fov_range)
        ex, ez = self.enemy_pt.x, self.enemy_pt.z

        for ang in angles:
            r = 3.5
            while r < max_r:
                ux = ex + r * np.sin(ang)
                uz = ez + r * np.cos(ang)
                i  = int((ux - self.unity_o[0]) / self.res)
                j  = int((uz - self.unity_o[1]) / self.res)
                if not (0 <= i < m.shape[0] and 0 <= j < m.shape[1]):
                    break
                m[i, j] = 1
                m[i+1, j] = 1
                m[i, j+1] = 1
                m[i-1, j] = 1
                m[i, j-1] = 1
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
        ###################################시각화#########################################
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
        ################################################################################
        # plan and publish path
        # 1) start, enemy 위치 필요
        if self.start_idx is None or self.enemy_pt is None:
            return

        sx = self.unity_o[0] + self.start_idx[0] * self.res
        sz = self.unity_o[1] + self.start_idx[1] * self.res
        ex, ez = self.enemy_pt.x, self.enemy_pt.z

        ####### 적 heading 어딨으유?
        start_world = [sx, sz, 0]
        start_world = np.array(start_world)
        h_s = 0
        origin = self.unity_o
        voxel_size = self.res
        nx, nz = self.nx, self.nz
        approach_min = self.approach_min
        goal_world = [ex, ez, 0]
        h_g = self.enemy_yaw # flask 출력 적 전차 헤딩 값
        h_g = unity_deg_to_math_rad(h_g)
        self.get_logger().info(f"aggggggggggggggggggggg {h_g}")
        occ2d_filled = masked
        start2d = self.start_idx
        goal2d = self.goal_idx

        ## 내 전차
        # 월드좌표에서 내전차 heading 벡터 반환
        hs_world  = sample_ray_cells(start_world[:2], heading_to_unit_vec(h_s),
                                    origin=origin, voxel_size=voxel_size,
                                    map_shape=(nx, nz), max_range_m=10.0,
                                    return_mode="world")

        ## 적 전차
        # 월드좌표에서 적 전차 heading 벡터 반환
        hg_world  = sample_ray_cells(goal_world[:2], heading_to_unit_vec(h_g),
                                    origin=origin, voxel_size=voxel_size,
                                    map_shape=(nx, nz), max_range_m=10.0,
                                    return_mode="world")

        # 적전차 heading +-90도 벡터
        h_g_v_left  = heading_to_unit_vec(h_g + np.pi / 2)   # world heading 기준 좌측 90° 벡터
        h_g_v_right = heading_to_unit_vec(h_g - np.pi / 2)   # world heading 우측 90° 벡터

        # world좌표계 기준 적전차 시야각 ray
        h_g_v_left_ray  = sample_ray_cells(goal_world[:2], h_g_v_left,# 좌측 시야각 선, 튜플(x, z)를 list로 묶어서 반환
                                    origin=origin, voxel_size=voxel_size,
                                    map_shape=(nx, nz), max_range_m=300.0,
                                    return_mode="world") 
        h_g_v_right_ray = sample_ray_cells(goal_world[:2], h_g_v_right,
                                    origin=origin, voxel_size=voxel_size,
                                    map_shape=(nx, nz), max_range_m=300.0,
                                    return_mode="world")   # 우측 시야각 선, 튜플(x, z)를 list로 묶어서 반환

        front_flag = is_front(sx, sz, ex, ez, h_g) # 내전차 우 전방, 적 전차 우 전방 헤딩

        if front_flag:
            # print("단차는 적 전차 시야각 전방에 위치한다!")
            # print("조중수, 경유점 생성해서 적 측면 공격할 것")
            
            # world 좌표에서 수선의 발 계산 # world 좌표에서 우, 전방
            F_left,  d_self_l, d_enemy_l  = foot_to_ray(start_world[:2], goal_world[:2], h_g_v_left, origin=origin, voxel_size=voxel_size, map_shape=(nx, nz))
            F_right, d_self_r, d_enemy_r  = foot_to_ray(start_world[:2], goal_world[:2], h_g_v_right, origin=origin, voxel_size=voxel_size, map_shape=(nx, nz))

            # 수선의 발 map 안쪽으로 clamp
            F_left_cl  = clamp_point_on_ray(
                F_left,  goal_world[:2], h_g_v_left,
                origin=origin, voxel_size=voxel_size, map_shape=(nx, nz))
            F_right_cl = clamp_point_on_ray(
                F_right, goal_world[:2], h_g_v_right,
                origin=origin, voxel_size=voxel_size, map_shape=(nx, nz))

            # world좌표에서 적 전차 기준 좌, 우에서 내 전차와 가까운 점 선택
            if np.linalg.norm(start_world[:2] - F_left_cl) <= np.linalg.norm(start_world[:2] - F_right_cl):
                F_sel       = F_left_cl
                d_enemy_sel = np.linalg.norm(goal_world[:2] - F_left_cl)
                sel_side    = "Left"
            else:
                F_sel       = F_right_cl
                d_enemy_sel = np.linalg.norm(goal_world[:2] - F_right_cl)
                sel_side    = "Right"
            # print(f"{sel_side} FOV 수선의 발(클램프) 좌표 : ({F_sel[0]:.3f}, {F_sel[1]:.3f}) m")
            # print(f"적 전차로부터  →  수선의 발 거리          : {d_enemy_sel:.3f} m")
            
            # 최초 경유점 시각화용
            r_sel, f_sel = world_to_index(F_sel, origin, voxel_size) # map index 우 전방
            
            
            # 예외처리 시작
            # 수선의 발이 최소 접근거리보다 작은 경우
            if(d_enemy_sel < approach_min): 
                u_sel = h_g_v_left / np.linalg.norm(h_g_v_left) if sel_side == "Left" else h_g_v_right / np.linalg.norm(h_g_v_right) # world 좌표에어 적전차 -> 수선의 발 까지 유닛 벡터
                
                # world 좌표 상 최소 접근거리로 경유점 갱신
                F_sel1 = goal_world[:2] + approach_min * u_sel

                # 지도 밖으로 나갈 수 있으니 지도 안으로 clamp
                F_sel1 = clamp_point_on_ray(
                    F_sel1, goal_world[:2], u_sel,
                    origin=origin, voxel_size=voxel_size, map_shape=(nx, nz)
                )
                r_sel1, f_sel1 = world_to_index(F_sel1, origin, voxel_size) # 수선의 발이 최소 접근거리 이내인 경우 지도 밖인지 판별하여 지도 안으로 보정
                
                if occ2d_filled[r_sel1, f_sel1] == 1:            # 수선의 발이 최소 접근거리 이내인 경우 지도 밖인지 판별하여 지도 안으로 보정, 장애물인 경우
                    print("경유점이 장애물!  FOV 선을 따라 재검색")

                    # 선택된 시야각(u_sel), 적에게 가까운 점 우선 탐색
                    t_start = np.dot(F_sel - goal_world[:2], u_sel)
                    P_new, idx_new = find_free_on_ray(goal_world[:2], u_sel, t_start,
                                                    binary_map=occ2d_filled,
                                                    origin=origin, voxel_size=voxel_size,
                                                    map_shape=(nx, nz))
                    # 실패하면 반대 시야각에서 적에게 가까운 점 우선 탐색
                    if P_new is None:
                        u_opp = h_g_v_right/np.linalg.norm(h_g_v_right) if sel_side=="Left" else h_g_v_left/np.linalg.norm(h_g_v_left)
                        _, _, t_max = foot_to_ray(goal_world[:2], goal_world[:2], u_opp,
                                                origin=origin, voxel_size=voxel_size,
                                                map_shape=(nx, nz))
                        P_new, idx_new = find_free_on_ray(goal_world[:2], u_opp, t_max,
                                                        search_outward=False,
                                                        binary_map=occ2d_filled,
                                                        origin=origin, voxel_size=voxel_size,
                                                        map_shape=(nx, nz))
                        if P_new is not None:
                            sel_side = "Right" if sel_side=="Left" else "Left"   # 방향 전환

                    # 성공 시 경유점 갱신
                    if P_new is not None:
                        F_sel2         = P_new # 수선의 발, 최소 접근거리 내부면 밀어내기, 장애물인 경우 적에게 가까운 점으로 갱신
                        r_sel2, f_sel2 = idx_new
                        d_enemy_sel2   = np.linalg.norm(goal_world[:2] - F_sel2)
                        # print(f"[재검색 성공] 새 경유점 ({sel_side}) : "
                            # f"{F_sel2[0]:.2f}, {F_sel2[1]:.2f} m  (적 거리 {d_enemy_sel2:.2f} m)")
                    else:
                        # print("두 방향 모두에서 자유 셀을 찾지 못했습니다 → 경유점 사용 포기")
                        pass
            
            # 수선의 발이 최소 접근거리 밖에 있는 경우
            else:
                if occ2d_filled[r_sel, f_sel] == 1:            # ← 막힌 셀 우 전방
                    print("경유점이 장애물!  FOV 선을 따라 재검색")

                    # 1) 선택된 시야각(u_sel) 방향으로 적에게 가까운 점 탐색
                    u_sel   = h_g_v_left/np.linalg.norm(h_g_v_left) if sel_side=="Left" else h_g_v_right/np.linalg.norm(h_g_v_right)
                    t_start = np.dot(F_sel - goal_world[:2], u_sel)
                    P_new, idx_new = find_free_on_ray(goal_world[:2], u_sel, t_start,
                                                    binary_map=occ2d_filled,
                                                    origin=origin, voxel_size=voxel_size,
                                                    map_shape=(nx, nz))
                    # 2) 실패하면 반대 시야각(u_opp)으로 적과 가까운 점 탐색
                    if P_new is None:
                        u_opp = h_g_v_right/np.linalg.norm(h_g_v_right) if sel_side=="Left" else h_g_v_left/np.linalg.norm(h_g_v_left)
                        _, _, t_max = foot_to_ray(goal_world[:2], goal_world[:2], u_opp,
                                                origin=origin, voxel_size=voxel_size,
                                                map_shape=(nx, nz))
                        P_new, idx_new = find_free_on_ray(goal_world[:2], u_opp, t_max,
                                                        search_outward=False,
                                                        binary_map=occ2d_filled,
                                                        origin=origin, voxel_size=voxel_size,
                                                        map_shape=(nx, nz))
                        if P_new is not None:
                            sel_side = "Right" if sel_side=="Left" else "Left"   # 방향 전환
                    
                    # 성공 시 경유점 갱신
                    # 수선의 발이 최소 접근범위 밖에 있으며 장애물인 경우 양방향 탐색 후 적에게 가까운 점 출력
                    if P_new is not None:
                        F_sel3         = P_new
                        r_sel3, f_sel3   = idx_new  # idx_new 우, 전방
                        # print('fffffff', r_sel3, f_sel3) # 우 전방
                        d_enemy_sel3   = np.linalg.norm(goal_world[:2] - F_sel3)
                        # print(f"[재검색 성공] 새 경유점 ({sel_side}) : "
                            # f"{F_sel3[0]:.2f}, {F_sel3[1]:.2f} m  (적 거리 {d_enemy_sel3:.2f} m)") # 우 전방
                    else:
                        # print("두 방향 모두에서 자유 셀을 찾지 못했습니다 → 경유점 사용 포기")
                        pass

        else:
            # print("내 전차는 적 전차의 후방에 있습니다.")
            # print("조종수, 최단거리로 공격할 것!")
            pass

        # ====================== 만들어진 경유점 정리 ======================
        # ── 경유점 인덱스(wp_idx) 확정 ────────────────────────────
        wp_idx = None
        if   "r_sel3" in locals() and r_sel3 is not None:   # 수선의 발이 최소 접근거리 밖, 장애물인 경우 적에게 가까운 점
            wp_idx = (r_sel3, f_sel3) # 우 전방
        elif "r_sel2" in locals() and r_sel2 is not None:   # 수선의 발이 최소 접근거리 내부인 경우 밀어내기, 장애물인 경우 적에게 가까운 점으로 갱신
            wp_idx = (r_sel2, f_sel2)
        elif "r_sel1" in locals() and r_sel1 is not None:   # 수선의 발이 최소 접근거리 이내인 경우 지도 밖인지 판별하여 지도 안으로 보정
            wp_idx = (r_sel1, f_sel1)
        elif "r_sel"  in locals() and r_sel  is not None:   # 최초 수선의 발
            wp_idx = (r_sel,  f_sel)

        k0 = self.k0
        # ====================== 2D A* 알고리즘 3회 실시 ======================
        path_s2w = []
        path_w2r = []
        path_r2g = []
        if wp_idx is not None:        # 경유점이 있을 때
            path_s2w = astar_2d(occ2d_filled, start2d, (wp_idx[0], wp_idx[1])) # 우 전방
            # wp_world = index_to_world((wp_idx[0], wp_idx[1]), origin, voxel_size)
            
            # 적 후방 잡기 위해 최소 접근거리 내 적 전방 기준 +- 150도 점 2개 지정하여 접근
            # wp_ring_w, wp_ring_idx = waypoint_on_approach_ring(
            #     O_xy       = goal_world[:2],
            #     heading    = h_g,
            #     radius     = approach_min,          # 50 m
            #     binary_map = occ2d_filled,
            #     origin     = origin,
            #     voxel_size = voxel_size,
            #     map_shape  = (nx, nz),
            #     prefer_xy  = (wp_idx[0], wp_idx[1]))       # 경유점 우, 전방
            wp_ring_w, wp_ring_idx = waypoint_on_approach_ring(
                O_xy       = goal_world[:2],       # 적 전차 위치 (x,z)
                heading    = h_g,                  # 적 전차 헤딩 (rad)
                radius     = approach_min,         # 접근 최소 거리 (예: 50m)
                binary_map = occ2d_filled,         # 장애물 맵 (2D binary)
                origin     = origin,               # 맵 원점 (월드 좌표)
                voxel_size = voxel_size,           # 맵 해상도
                map_shape  = (nx, nz),             # 맵 크기 (row, col)
                prefer_xy  = (wp_idx[0], wp_idx[1]) # 내 전차 경유점 인덱스 (2D)
            )

            self.get_logger().info(f"rarararaarararra {wp_ring_idx}")
            #self.get_logger().info(f"qqqqqqqqqqqqqqqqqqqqqqqq {wp_idx[0]} , {wp_idx[1]}")
            #self.get_logger().info(f"999999999999999999 {goal2d}")
            path_w2r = astar_2d(occ2d_filled, (wp_idx[0], wp_idx[1]),  (wp_ring_idx[0], wp_ring_idx[1]))
            
            path_r2g = astar_2d(occ2d_filled, (wp_ring_idx[0], wp_ring_idx[1]), goal2d)

            path_full2 = path_s2w[:-1] + path_w2r[:-1] + path_r2g[:-1]
            self.get_logger().info(f"eeeeeeee {path_w2r[0]}{path_w2r[1]}")
            self.get_logger().info(f"ffffffff {path_r2g}")


            #(wp_idx[0], wp_idx[1]),  (wp_ring_idx[0], wp_ring_idx[1])
            self.get_logger().info(f"aaaaaaaaa {start2d}{wp_idx[0]}{wp_idx[1]}{wp_ring_idx[0]}{wp_ring_idx[1]}{goal2d}")
        else:                          # 경유점이 없으면 한 번만
            path_full = astar_2d(occ2d_filled, start2d, goal2d)

            self.get_logger().info(f"bbbbbbbbbbbbbbbbbb {start2d}{goal2d}")
            ##(wp_idx[0], wp_idx[1]),  (wp_ring_idx[0], wp_ring_idx[1])


        if path_s2w or path_w2r or path_r2g or path_full:
            # print('2D 경로 찾음')
            try:
                path_2d_arr = np.array(path_full2, dtype=int)  # shape = (N, 2)
                # print('hi', path_2d_arr)
                self.get_logger().info(f"cccccccc {path_2d_arr[0]}{path_2d_arr[-1]}")
            except:
                path_2d_arr = np.array(path_full, dtype=int)  # shape = (N, 2)
                # print('go', path_2d_arr)
                self.get_logger().info(f"ddddddddd {path_2d_arr[0]}{path_2d_arr[-1]}")
                
            k0_col = np.full((path_2d_arr.shape[0], 1), 0, dtype=int)
            for idx in range(len(path_2d_arr)):
                x1, z1 = path_2d_arr[idx]
                idxs = np.where(self.ogm[x1, z1, :])[0]
                if idxs.size:                       # True가 하나 이상 존재
                    k0_col[idx] = idxs.max() + k0
                else:                               # True가 없을 때의 처리
                    k0_col[idx] = k0_col[idx-1]         # 없는 경우 1개 이전 값을 활용
                
                # k0_col[idx] = np.max(np.where(ogm[x1, z1, :])[0]) + k0
            path_arr_3d = np.hstack((path_2d_arr, k0_col))
            
            ros_pts = []                                                      # path_arr_3d를 월드 좌표로 변환
            for idx in path_arr_3d:
                i, k, j = idx
                # 셀의 중앙 좌표 = origin + (index + 0.5)*voxel_size
                x = origin[0] + (i) * voxel_size
                z = origin[1] + (k) * voxel_size
                y = origin[2] + (j + 0.5) * voxel_size
                #path_world_3d.append((x, z, y))
                p = unity_point_to_ros({'x': x, 'z': z, 'y': y})
                ros_pts.append((p.x, p.y, p.z))

            self.path_pts = np.array(ros_pts)
            self.need_replan = False
            # print("3차원공간의 3D 경로 인덱스:", path_arr_3d)
        else:
            # print("2D에서 경로를 찾지 못했습니다."
            pass



        # # 3) 적→아군 방향 단위벡터
        # dx, dz = sx - ex, sz - ez
        # dist = np.hypot(dx, dz)
        # if dist < 1e-3:
        #     self.get_logger().warn("아군과 적전차 위치가 너무 가깝습니다. 목표점 설정 불가")
        #     return

        # # 4) 최소~최대 거리 범위 내에서 장애물 없는 첫 후보 탐색
        # found = False
        # for r in np.arange(self.min_goal_dist, self.max_goal_dist + self.res, self.res):
        #     cx = ex + dx/dist * r
        #     cz = ez + dz/dist * r
        #     ix = int((cx - self.unity_o[0]) / self.res)
        #     iz = int((cz - self.unity_o[1]) / self.res)
        #     # 맵 범위 체크
        #     if not (0 <= ix < self.occ2d_orig.shape[0] and 0 <= iz < self.occ2d_orig.shape[1]):
        #         continue
        #     # 장애물 없는 셀(0)인지 확인
        #     if masked[ix, iz] == 0:
        #         self.goal_idx = (ix, iz)
        #         found = True
        #         break
        # if not found:
        #     self.get_logger().warn(
        #         f"{self.min_goal_dist}m~{self.max_goal_dist}m 범위 내 장애물 없는 목표점을 찾지 못했습니다."
        #     )
        #     return
        # -------------------------------------------------------

        # if self.need_replan:
        #     path = astar_2d(masked, self.start_idx, self.goal_idx)
        #     self.get_logger().info(f"masked num : {np.count_nonzero(masked)}")
        #     if not path:
        #         self.get_logger().error("A* 경로 생성 실패: 생성실패")
        #         return
        #     ros_pts = []
        #     for x, z in path:
        #         zs = np.where(self.ogm[x, z, :])[0]
        #         k = np.max(zs) if zs.size else 0
        #         wx = self.unity_o[0] + x * self.res
        #         wz = self.unity_o[1] + z * self.res
        #         wy = self.unity_o[2] + (k + 0.5) * self.res
        #         p = unity_point_to_ros({'x': wx, 'z': wz, 'y': wy})
        #         ros_pts.append((p.x, p.y, p.z))
        #     self.path_pts = np.array(ros_pts)
        #     self.need_replan = False
        if wp_idx is not None:
            wp_marker = Marker(
                header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
                ns='waypoint_marker',
                id=10,
                type=Marker.SPHERE,
                action=Marker.ADD,
                pose=Pose(),
                scale=Vector3(x=5.0, y=5.0, z=5.0),
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            )
            wx = self.unity_o[0] + (wp_idx[0] + 0.5) * self.res
            wz = self.unity_o[1] + (wp_idx[1] + 0.5) * self.res
            wy = self.unity_o[2]
            p = unity_point_to_ros({'x': wx, 'z': wz, 'y': wy})
            wp_marker.pose.position.x = p.x
            wp_marker.pose.position.y = p.y
            wp_marker.pose.position.z = p.z
            self.pub_wp_marker.publish(wp_marker)  # Marker 메시지로 publish

        if 'wp_ring_idx' in locals() and wp_ring_idx is not None:
            wp_ring_marker = Marker(
                header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
                ns='waypoint_ring_marker',
                id=11,
                type=Marker.SPHERE,
                action=Marker.ADD,
                pose=Pose(),
                scale=Vector3(x=5.0, y=5.0, z=5.0),
                color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            )
            wx = self.unity_o[0] + (wp_ring_idx[0] + 0.5) * self.res
            wz = self.unity_o[1] + (wp_ring_idx[1] + 0.5) * self.res
            wy = self.unity_o[2]
            p = unity_point_to_ros({'x': wx, 'z': wz, 'y': wy})
            wp_ring_marker.pose.position.x = p.x
            wp_ring_marker.pose.position.y = p.y
            wp_ring_marker.pose.position.z = p.z
            self.pub_wp_ring_marker.publish(wp_ring_marker)  # Marker 메시지로 publish

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
