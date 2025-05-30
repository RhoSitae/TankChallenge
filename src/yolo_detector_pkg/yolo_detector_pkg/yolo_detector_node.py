#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
import cv2
from ultralytics import YOLO
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path', '/path/to/best1.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.latest_camera_info = None
        self.mid_x = 0.0
        self.mid_y = 0.0

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Subscribe to raw image and camera info
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            qos
        )
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            qos
        )

        # Publishers for annotated image, camera info, and markers
        self.pub_img = self.create_publisher(
            Image,
            '/yolo_detector/image_bbox',
            qos
        )
        self.pub_info = self.create_publisher(
            CameraInfo,
            '/yolo_detector/camera_info',
            qos
        )
        self.pub_marker = self.create_publisher(
            MarkerArray,
            '/yolo_detector/markers',
            qos
        )

        self.pub_center = self.create_publisher(PointStamped, '/yolo_detector/center_point', qos)

        self.target_classes = {0: "person", 2: "car", 7: "truck", 15: "rock", 80:"front_side", 81:"in_100"}

        self.get_logger().info(f'Loaded YOLOv8 model: {model_path}')

    def info_callback(self, msg: CameraInfo):
        # Cache latest camera calibration
        self.latest_camera_info = msg

    def callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)[0]


        markers = MarkerArray()
        idx = 0
        for idx, box in enumerate(results.boxes):
            cls_id = int(box.cls[0])
            if cls_id in self.target_classes:

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]); conf = float(box.conf[0])

                # Draw bounding box on image
                label = f"{self.model.names[cls]}:{conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                # # 1) 사각형 테두리 LINE_LIST
                # m = Marker()
                # m.header = msg.header
                # m.ns = 'yolo_bbox'
                # m.id = idx * 2    # 짝수 ID
                # m.type = Marker.LINE_LIST
                # m.action = Marker.ADD
                # m.scale.x = 0.005
                # m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
                # pts = [
                #     Point(x=float(x1), y=float(y1), z=0.0),
                #     Point(x=float(x2), y=float(y1), z=0.0),
                #     Point(x=float(x2), y=float(y1), z=0.0),
                #     Point(x=float(x2), y=float(y2), z=0.0),
                #     Point(x=float(x2), y=float(y2), z=0.0),
                #     Point(x=float(x1), y=float(y2), z=0.0),
                #     Point(x=float(x1), y=float(y2), z=0.0),
                #     Point(x=float(x1), y=float(y1), z=0.0),
                # ]
                # m.points = pts
                # markers.markers.append(m)


                mid_x = (x1 + x2) / 2.0
                mid_y = (y1 + y2) / 2.0

                center_msg = PointStamped()
                center_msg.header = msg.header           # 이미지 헤더 그대로
                center_msg.point.x = float(mid_x)        # 꼭 float!
                center_msg.point.y = float(mid_y)
                center_msg.point.z = 0.0
                # (2) 퍼블리시
                self.pub_center.publish(center_msg)
                # # 2) POINTS 타입 마커 생성
                # pt_m = Marker()
                # pt_m.header = msg.header
                # pt_m.ns     = 'yolo_center_point'
                # pt_m.id     = idx  # idx 가 유일하다면 그대로 써도 OK
                # pt_m.type   = Marker.POINTS
                # pt_m.action = Marker.ADD

                # # POINTS 에선 scale.x, scale.y 가 점 크기 (픽셀 단위가 아니라 RViz 단위)
                # pt_m.scale.x = 2.0  
                # pt_m.scale.y = 2.0 

                # # 원하는 색으로
                # pt_m.color.r = 1.0
                # pt_m.color.g = 0.0
                # pt_m.color.b = 0.0
                # pt_m.color.a = 1.0

                # 꼭 float 로!
                # pt_m.points = [ Point(x=float(mid_x), y=float(mid_y), z=0.0) ]

                # markers.markers.append(pt_m)

                break
                # idx += 1

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
        #self.pub_center.publish(center_msg)
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