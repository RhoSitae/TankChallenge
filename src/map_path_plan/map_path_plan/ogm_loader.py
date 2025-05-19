import struct
import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from scipy import ndimage
from scipy.ndimage import binary_erosion, binary_dilation
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Point
from tank_package.common.coord_utils import unity_point_to_ros
from tank_package_msg.msg import TankState, EnemyState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time

# 기존 astar_2d 함수 (순수 2D)
def astar_2d(occ2d, start, goal):
    nx, ny = occ2d.shape
    neigh = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start,goal), 0, start, None)]
    came, gscore = {}, {start:0}
    while open_set:
        f, g, cur, parent = heapq.heappop(open_set)
        if cur in came: continue
        came[cur] = parent
        if cur == goal:
            path=[]
            n=cur
            while n:
                path.append(n)
                n=came[n]
            return path[::-1]
        for dx, dy in neigh:
            nb = (cur[0]+dx, cur[1]+dy)
            if not (0<=nb[0]<nx and 0<=nb[1]<ny): continue
            if occ2d[nb]: continue
            ng = g+1
            if ng < gscore.get(nb, float('inf')):
                gscore[nb] = ng
                heapq.heappush(open_set, (ng+h(nb,goal), ng, nb, cur))
    return []

class OGMAndPathLoader(Node):
    def __init__(self):
        super().__init__('ogm_and_path_loader')

        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 파라미터
        npz_path = self.declare_parameter('npz_path', 'flat_and_hills_whole_OGM(0.5)_with_meta.npz').value
        slope_deg = float(self.declare_parameter('slope_threshold_deg', 30.0).value)
        ker_e = int(self.declare_parameter('erosion_kernel', 3).value)
        ker_d = int(self.declare_parameter('dilation_kernel', 8).value)

        # npz 로드
        meta = np.load(npz_path)
        self.ogm = meta['data']   # (nx, ny, nz)
        self.unity_o = meta['origin']
        self.res = float(meta['resolution'])

        # 2D OGM 처리: 높이 맵 및 경사도 이용
        occ3d = self.ogm.astype(bool)
        idx_max = np.argmax(occ3d, axis=2)
        self.height_map = idx_max * self.res

        Hmin = np.nanmin(self.height_map)
        Hf = np.where(np.isnan(self.height_map), Hmin, self.height_map)
        gx = ndimage.sobel(Hf, axis=0, mode='nearest') / (8*self.res)
        gz = ndimage.sobel(Hf, axis=1, mode='nearest') / (8*self.res)
        slope = np.arctan(np.hypot(gx, gz))
        occ2d = (slope > np.deg2rad(slope_deg)).astype(np.int8)

        occ2d = binary_erosion(occ2d, structure=np.ones((ker_e, ker_e), bool)).astype(np.int8)
        occ2d = binary_dilation(occ2d, structure=np.ones((ker_d, ker_d), bool)).astype(np.int8)
        self.occ2d = occ2d

        # 목표점 고정
        self.x_g, self.z_g, self.y_g = 280, 280, 3
        self.goal_world = np.array([self.x_g, self.z_g, self.y_g])
        self.goal_idx = tuple(((self.goal_world - self.unity_o) / self.res).astype(int))[:2]

        # 시작점 초기화 (TankState 콜백에서 갱신)
        self.start_idx = None

        # 구독자 설정
        self.create_subscription(TankState, "/tank_state", self.cb_tank, qos)
        self.create_subscription(EnemyState, "/enemy_state", self.cb_enemy, qos)  # 필요 시 목표 갱신용

        # 퍼블리셔 설정
        self.pub_path = self.create_publisher(Marker, "/global_path", qos)

        # 1초 주기 타이머
        self.create_timer(1.0, self.update_path)

    def cb_tank(self, msg: TankState):
        # TankState에서 위치 받아 시작점 갱신 (Y축 절대값으로 X 좌표, X축은 Z좌표로 변환 예시)
        self.start_world = np.array([abs(msg.position.y), msg.position.x, msg.position.z])
        self.start_idx = tuple(((self.start_world - self.unity_o) / self.res).astype(int))[:2]
        self.get_logger().info(f'Updated start_idx: {self.start_idx}')

    def cb_enemy(self, msg: EnemyState):
        # 필요 시 목표점 갱신 가능 (기본 동작은 로그 출력)
        self.get_logger().info(f"EnemyState received: pos.x={msg.position.x}, pos.y={msg.position.y}, pos.z={msg.position.z}")

    def update_path(self):
        if self.start_idx is None or self.goal_idx is None:
            self.get_logger().warn("Start or Goal index not set yet.")
            return

        self.get_logger().info(f"Computing path from {self.start_idx} to {self.goal_idx}")
        path = astar_2d(self.occ2d, self.start_idx, self.goal_idx)
        if not path:
            self.get_logger().warn("A* 경로를 찾지 못했습니다.")
            return

        path_2d_arr = np.array(path, dtype=int)

        # 높이 k0 계산
        k0 = 0
        k0 = int(k0 / self.res)

        # 3D 경로 인덱스 생성 (x, y, height)
        k0_col = np.full((path_2d_arr.shape[0], 1), 0, dtype=int)
        for idx in range(len(path_2d_arr)):
            x1, z1 = path_2d_arr[idx]
            indices = np.where(self.ogm[x1, z1, :])[0]
            if indices.size == 0:
                k0_col[idx] = 0 + k0
            else:
                k0_col[idx] = np.max(indices) + k0
        path_arr_3d = np.hstack((path_2d_arr, k0_col))

        path_world_3d = []
        for idx in path_arr_3d:
            i, k, j = idx
            x = self.unity_o[0] + (i) * self.res
            z = self.unity_o[1] + (k) * self.res
            y = self.unity_o[2] + (j + 0.5) * self.res
            path_world_3d.append((x, z, y))

        m = Marker(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            ns='global_path', id=0, type=Marker.LINE_STRIP,
            scale=Vector3(x=1.0, y=0.0, z=0.0),
            color=ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        )
        for (x, z, y) in path_world_3d:
            p = unity_point_to_ros({'x': x, 'z': z, 'y': y})
            m.points.append(Point(x=p.x, y=p.y, z=p.z))

        self.pub_path.publish(m)
        self.get_logger().info(f"Published path with {len(path_world_3d)} points")

def main(args=None):
    rclpy.init(args=args)
    node = OGMAndPathLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
