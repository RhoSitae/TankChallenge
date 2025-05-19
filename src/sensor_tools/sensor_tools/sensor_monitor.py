#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import csv
import matplotlib.pyplot as plt
from tank_package_msg.msg import TankState, EnemyState, LidarScan
from sensor_msgs.msg import Image
import signal


class SensorMonitor(Node):
    def __init__(self):
        signal.signal(signal.SIGINT, lambda *args: self.destroy_node())
        super().__init__('sensor_monitor')
        # 모니터링할 토픽과 CSV 파일명 설정
        self.topics = {
            '/tank_state':        {'last': None, 'dts': [], 'csv': 'tank_state.csv'},
            '/enemy_state':       {'last': None, 'dts': [], 'csv': 'enemy_state.csv'},
            '/lidar_scan':        {'last': None, 'dts': [], 'csv': 'lidar_scan.csv'},
            '/camera/image_raw':  {'last': None, 'dts': [], 'csv': 'camera_image_raw.csv'},
        }
        # CSV 파일 오픈 및 헤더 작성
        self.csv_files = {}
        self.csv_writers = {}
        for topic, info in self.topics.items():
            f = open(info['csv'], 'w', newline='')
            writer = csv.writer(f)
            writer.writerow(['index', 'dt_s', 'hz'])
            self.csv_files[topic] = f
            self.csv_writers[topic] = writer

        # 토픽 구독 설정
        self.create_subscription(TankState,  '/tank_state',       self.cb('/tank_state'),       10)
        self.create_subscription(EnemyState, '/enemy_state',      self.cb('/enemy_state'),      10)
        self.create_subscription(LidarScan,  '/lidar_scan',       self.cb('/lidar_scan'),       10)
        self.create_subscription(Image,      '/camera/image_raw', self.cb('/camera/image_raw'), 10)

    def cb(self, topic):
        def _cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            info = self.topics[topic]
            if info['last'] is not None:
                dt = now - info['last']
                info['dts'].append(dt)
                hz = 1.0 / dt if dt > 0 else float('inf')
                idx = len(info['dts'])
                # 콘솔 출력
                self.get_logger().info(f"{topic} → dt={dt*1000:.1f}ms ({hz:.1f} Hz)")
                # CSV 로깅
                self.csv_writers[topic].writerow([idx, f"{dt:.6f}", f"{hz:.2f}"])
            info['last'] = now
        return _cb

    def destroy_node(self):
        # CSV 파일 닫기
        for f in self.csv_files.values():
            f.close()
        # 주기 그래프 저장
        for topic, info in self.topics.items():
            if not info['dts']:
                continue
            plt.figure()
            plt.plot(info['dts'])
            plt.title(f"{topic} Callback Period")
            plt.xlabel("Message Index")
            plt.ylabel("Period (s)")
            plt.grid(True)
            fname = topic.strip('/').replace('/', '_') + '_period.png'
            plt.savefig(fname)
        plt.show()
        super().destroy_node()


def main():
    rclpy.init()
    node = SensorMonitor()

    # 종료 키 입력 스레드
    def wait_for_exit():
        input("Press Enter to stop monitoring and save data...\n")
        node.get_logger().info("Exit command received, shutting down...")
        rclpy.shutdown()
    threading.Thread(target=wait_for_exit, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down → logging CSV and plotting...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
