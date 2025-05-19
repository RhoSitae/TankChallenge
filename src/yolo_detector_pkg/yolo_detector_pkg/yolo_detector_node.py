#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import cv2
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path', '/path/to/best.pt')
        model_path = self.get_parameter('model_path')\
                         .get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.latest_camera_info = None

        # Subscribe to raw image and camera info
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        # Publishers for annotated image, camera info, and markers
        self.pub_img = self.create_publisher(
            Image,
            '/yolo_detector/image_bbox',
            10
        )
        self.pub_info = self.create_publisher(
            CameraInfo,
            '/yolo_detector/camera_info',
            10
        )
        self.pub_marker = self.create_publisher(
            MarkerArray,
            '/yolo_detector/markers',
            10
        )

        self.get_logger().info(f'Loaded YOLOv8 model: {model_path}')

    def info_callback(self, msg: CameraInfo):
        # Cache latest camera calibration
        self.latest_camera_info = msg

    def callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)[0]

        markers = MarkerArray()
        idx = 0
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0]); conf = float(box.conf[0])

            # Draw bounding box on image
            label = f"{self.model.names[cls]}:{conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, label, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # Create 2D LINE_LIST marker for RViz Camera display
            m = Marker()
            m.header = msg.header
            m.ns = 'yolo'
            m.id = idx
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.01
            m.color.a = 1.0; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0

            pts = [
                Point(x=float(x1), y=float(y1), z=0.0), Point(x=float(x2), y=float(y1), z=0.0),
                Point(x=float(x2), y=float(y1), z=0.0), Point(x=float(x2), y=float(y2), z=0.0),
                Point(x=float(x2), y=float(y2), z=0.0), Point(x=float(x1), y=float(y2), z=0.0),
                Point(x=float(x1), y=float(y2), z=0.0), Point(x=float(x1), y=float(y1), z=0.0),
            ]
            m.points = pts
            markers.markers.append(m)
            idx += 1

        # Publish annotated image with original header
        out_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)

        # Republish camera info with matching header
        if self.latest_camera_info is not None:
            cam_info = self.latest_camera_info
            cam_info.header = out_img.header
            self.pub_info.publish(cam_info)

        # Publish MarkerArray for Camera display
        self.pub_marker.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()