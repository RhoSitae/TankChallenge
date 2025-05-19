#!/usr/bin/env python3
import os
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from tank_package.common.coord_utils import unity_point_to_ros
from tank_package_msg.msg import TankState, EnemyState

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
            path=[]; n=cur
            while n:
                path.append(n)
                n = came[n]
            return path[::-1]
        for dx, dy in neigh:
            nb = (cur[0]+dx, cur[1]+dy)
            if not (0<=nb[0]<nx and 0<=nb[1]<ny): continue
            if occ2d[nb]: continue
            ng = g+1
            if ng < gscore.get(nb, np.inf):
                gscore[nb] = ng
                heapq.heappush(open_set, (ng + h(nb,goal), ng, nb, cur))
    return []

class PathLoader(Node):
    def __init__(self):
        super().__init__('path_loader')
        self.get_logger().info('PathLoader 초기화 중...')

        # NPZ 파일 경로 파라미터 (패키지 공유 디렉토리 기준)
        default_npz = os.path.join(
            get_package_share_directory('map_path_plan'),
            'flat_and_hills_whole_OGM(0.5)_with_meta.npz'
        )
        npz_path = self.declare_parameter('npz_path', default_npz).value

        # 1) NPZ 로드
        try:
            meta = np.load(npz_path)
        except Exception as e:
            self.get_logger().error(f'NPZ 파일 로드 실패: {npz_path} → {e}')
            raise SystemExit(1)

        ogm3d    = meta['data']       # (nx, ny, nz)
        unity_o  = meta['origin']     # [ox, oy, oz]
        self.res = float(meta['resolution'])
        # (i,j)별 첫 occupied k 인덱스
        self.k_idx = np.argmax(ogm3d>0, axis=2)
        self.unity_o = unity_o

        # 2) 구독: 2D OGM Marker, TankState, EnemyState
        self.sub_map   = self.create_subscription(
            Marker, '/ogm_2d_markers', self.cb_map, 10)
        self.sub_tank  = self.create_subscription(
            TankState, '/tank_state', self.cb_tank, 10)
        self.sub_enemy = self.create_subscription(
            EnemyState, '/enemy_state', self.cb_enemy, 10)

        # 3) 퍼블리셔: 글로벌 경로 (LINE_STRIP)
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability  = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        self.pub_path = self.create_publisher(Marker, '/global_path', qos)

        # 시작/목표 인덱스 초기화
        self.start_idx = None
        self.goal_idx  = None

        self.get_logger().info('PathLoader 준비 완료.')

    def cb_tank(self, msg: TankState):
        # unity→ros 된 msg.position 을 인덱스로 변환
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        i = int((x - self.unity_o[0]) / self.res)
        j = int((z - self.unity_o[1]) / self.res)
        self.start_idx = (i, j)
        self.get_logger().info(f'Start index 설정: {self.start_idx}')

    def cb_enemy(self, msg: EnemyState):
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        i = int((x - self.unity_o[0]) / self.res)
        j = int((z - self.unity_o[1]) / self.res)
        self.goal_idx = (i, j)
        self.get_logger().info(f'Goal index 설정: {self.goal_idx}')

    def cb_map(self, marker: Marker):
        # CUBE_LIST → 2D 이진 맵 복원
        res = marker.scale.x
        origin_pt = marker.points[0]
        xs = [p.x for p in marker.points]
        zs = [p.z for p in marker.points]
        nx = int(round((max(xs) - origin_pt.x) / res)) + 1
        ny = int(round((max(zs) - origin_pt.z) / res)) + 1
        occ2d = np.zeros((nx, ny), dtype=np.int8)
        for p in marker.points:
            i = int((p.x - origin_pt.x) / res)
            j = int((p.z - origin_pt.z) / res)
            occ2d[i, j] = 1

        # 시작/목표 인덱스 유효하면 A* 실행
        if self.start_idx is None or self.goal_idx is None:
            self.get_logger().info('waiting for start_idx and goal_idx...')
            return

        path2d = astar_2d(occ2d, self.start_idx, self.goal_idx)
        if not path2d:
            self.get_logger().warn('A* 경로를 찾지 못했습니다.')
            return

        # 3D LINE_STRIP Marker 생성
        m = Marker(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            ns='global_path', id=0,
            type=Marker.LINE_STRIP,
            scale=Vector3(x=1.0, y=0.0, z=0.0),
            color=ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
        )
        for (i, j) in path2d:
            k = self.k_idx[i, j]
            ux = self.unity_o[0] + (i+0.5)*self.res
            uz = self.unity_o[1] + (j+0.5)*self.res
            uy = self.unity_o[2] + (k+0.5)*self.res
            ros_pt = unity_point_to_ros({'x':ux,'z':uz,'y':uy})
            m.points.append(Point(x=ros_pt.x, y=ros_pt.y, z=ros_pt.z))

        self.pub_path.publish(m)
        self.get_logger().info(f'3D 경로 퍼블리시: {len(path2d)} 포인트')

def main():
    rclpy.init()
    node = PathLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
