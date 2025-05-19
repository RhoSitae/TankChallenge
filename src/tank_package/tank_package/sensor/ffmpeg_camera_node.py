#!/usr/bin/env python3
import subprocess, threading, queue, time, signal
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

RTMP_URL = "rtmp://192.168.0.3/live/stream_key"
WIDTH, HEIGHT, FPS = 1920, 1080, 60

class FfmpegCameraNode(Node):
    def __init__(self):
        super().__init__('ffmpeg_camera')
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.frame_q = queue.Queue(maxsize=10)  # 큐 크기 확장

        # FFmpeg 실행
        cmd = [
            'ffmpeg', '-loglevel', 'warning',
            '-fflags', 'nobuffer', '-flags', 'low_delay',
            '-i', RTMP_URL,
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-r', str(FPS),  # FPS 설정
            '-vsync', '2',  # vsync 설정 (프레임 동기화)
            'pipe:1'
        ]
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                      bufsize=WIDTH*HEIGHT*3*2,
                                      preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))

        # 스레드 시작
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _capture_loop(self):
        bytes_per_frame = WIDTH * HEIGHT * 3
        while rclpy.ok():
            raw = self._proc.stdout.read(bytes_per_frame)
            if len(raw) < bytes_per_frame:
                time.sleep(0.01)
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
            if self.frame_q.qsize() < 10:  # 큐가 가득 차지 않으면 계속 받아오기
                self.frame_q.put(frame)
        self.get_logger().info('Capture loop exiting')

    def _publish_loop(self):
        rate = self.create_rate(FPS)
        while rclpy.ok():
            try:
                frame = self.frame_q.get(timeout=1.0)
            except queue.Empty:
                continue
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.img_pub.publish(img_msg)
            rate.sleep()
        self.get_logger().info('Publish loop exiting')

    def shutdown_ffmpeg(self):
        if self._proc.poll() is None:
            self.get_logger().info('Shutting down FFmpeg')
            self._proc.send_signal(signal.SIGINT)
            try:
                self._proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()


def main():
    rclpy.init()
    node = FfmpegCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_ffmpeg()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
