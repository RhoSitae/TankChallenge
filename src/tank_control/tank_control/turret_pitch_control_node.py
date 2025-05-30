#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Float32
from tank_package_msg.msg import TankState
from tank_control.pid_controller import PIDDegController_non_limit
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from functools import partial

from tank_control.ballistic_computer import BallisticCalculator

def unity_deg_to_math_rad(angle_deg):
    deg_math = 90.0 - angle_deg
    rad = np.deg2rad(deg_math)
    return (rad + np.pi) % (2*np.pi) - np.pi  # normalize to [-π, π)

class TurretPitchControlNode(Node):
    def __init__(self):
        super().__init__('turret_pitch_control_node')

        # QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 파라미터 선언 (PID + 목표각 + FF gain)
        for name, default in [
            ('kp', 4.1), ('ki', 0.0), ('kd', 0.05),
            ('dt', 0.1), ('target_angle', 0.0),('target_pixel', 960),
            ('ff_gain', 1.0), ('case', 0)
        ]:

            self.declare_parameter(name, default)

        # 초기 파라미터 값 로드
        kp           = self.get_parameter('kp').value
        ki           = self.get_parameter('ki').value
        kd           = self.get_parameter('kd').value
        dt           = self.get_parameter('dt').value
        self.target_angle = self.get_parameter('target_angle').value
        self.target_pixel = self.get_parameter('target_pixel').value
        self.ff_gain      = self.get_parameter('ff_gain').value
        self.case = self.get_parameter('case').value

        # 탄도 계산기 생성
        self.bcm = BallisticCalculator('/home/strho/tank_ros2_ws/src/tank_control/tank_control/firing_tables.csv')

        # PID 컨트롤러 생성
        self.pid_controller = PIDDegController_non_limit(kp=kp, ki=ki, kd=kd, dt=dt)

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self._on_param_change)

        # 내부 상태 초기화
        self.move_rf     = 'R'
        self.rf_weight   = 0.0
        self.current_angle = 0.0
        self.error = 0.0
        self.p_center_x = 0.0
        self.p_center_y = 0.0
        # 구독: TankState
        self.create_subscription(
            TankState, '/tank_state',
            self.state_callback,
            qos
        )
        # 구독: steering_command
        self.create_subscription(
            String, '/steering_command',
            partial(self.command_callback, 'moveQE'),
            qos
        )
        self.create_subscription(
            PointStamped, '/yolo_detector/center_point',
            self.center_point_callback,
            qos
        )
        self.create_subscription(
            TankState, '/tank_state',
            self.state_callback,
            qos
        )
        # 퍼블리시: turret pitch 명령
        self.pub_pitch_cmd    = self.create_publisher(String,  '/turret_pitch_control',     qos)
        self.pub_current_angle = self.create_publisher(Float32, '/turret_pitch/current_angle', qos)
        self.pub_target_angle  = self.create_publisher(Float32, '/turret_pitch/target_angle', qos)
        self.pub_image_center  = self.create_publisher(Float32, '/turret_pitch/image_center',qos)
        self.get_logger().info(f'TurretPitchControlNode started')

    def _on_param_change(self, params):
        updated = []
        for p in params:
            if p.name == 'kp':
                self.pid_controller.kp = p.value; updated.append('kp')
            elif p.name == 'ki':
                self.pid_controller.ki = p.value; updated.append('ki')
            elif p.name == 'kd':
                self.pid_controller.kd = p.value; updated.append('kd')
            elif p.name == 'dt':
                self.pid_controller.dt = p.value; updated.append('dt')
            elif p.name == 'target_angle':
                self.target_angle = p.value; updated.append('target_angle')
            elif p.name == 'target_pixel':
                self.target_pixel = p.value; updated.append('target_pixel')
            elif p.name == 'ff_gain':
                self.ff_gain = p.value; updated.append('ff_gain')
            elif p.name == 'case':
                self.case = p.value; updated.append('case')
                    
        return SetParametersResult(successful=True)

    def command_callback(self, topic_key: str, msg: String):
        """ '/steering_command' 메시지를 받아 feed-forward 파라미터 갱신 """
        try:
            direction, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().error(f"Invalid steering format: '{msg.data}'")
            return

        # feed-forward용 내부 상태에 저장
        self.move_rf   = direction
        self.rf_weight = weight
        # self.get_logger().info(f"[{topic_key}] ← {direction},{weight:.3f}")
    def center_point_callback(self, msg: PointStamped):
        self.p_center_x = msg.point.x
        self.p_center_y = msg.point.y


    def state_callback(self, msg: TankState):
        """ TankState 수신 시 PID + FF 계산하여 turret pitch 명령 퍼블리시 """
        direction = ''
        weight = 0.0
        a_weight = 0.0

        # 현재 body 각도 (deg)
        current_body_angle = np.deg2rad(msg.body_euler.y + 360)

        # 현재 turret 각도 (deg)
        self.current_angle = - msg.turret_euler.y
        pitch_rad_current    = np.deg2rad(self.current_angle)
        curr_math_rad = (pitch_rad_current + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화

        pitch_rad_target     = np.deg2rad(self.target_angle)
        target_math_rad = (pitch_rad_target + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화
        
        # 현재 이미지 중심 픽셀
        self.pub_image_center.publish(Float32(data=self.p_center_x))

        if(self.case == 0):
            pass

        elif(self.case == 1): 
            self.pid_controller.kp = 2.0
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.05
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 1.0

            pid_output, self.error = self.pid_controller.compute(target_math_rad, curr_math_rad)
            
            # if self.move_rf == 'R':
            #     pid_output -= self.ff_gain * self.rf_weight
            # else:
            #     pid_output -= self.ff_gain * self.rf_weight

            if(pid_output>0):
                direction = 'R'
                weight = pid_output
            else:
                direction = 'F'
                weight = -pid_output

        elif(self.case == 2): # 거리 기반 탄도 계산
            self.pid_controller.kp = 2.0
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.05
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 1.0

            cb_elevation = self.bcm.get_elevation_for_range(msg.distance)

            pid_output, self.error = self.pid_controller.compute(unity_deg_to_math_rad(cb_elevation), curr_math_rad)

            if(pid_output>0):
                direction = 'R'
                weight = pid_output
            else:
                direction = 'F'
                weight = -pid_output


        # 차체 기준 전방 오일러각 target 각도 추종  + 전향제어 제
        # elif(self.case == 2):
        #     self.pid_controller.kp = 4.1
        #     self.pid_controller.ki = 0.0
        #     self.pid_controller.kd = 0.05
        #     self.pid_controller.dt = 0.1
        #     self.pid_controller.ff_gain = 0.0

        #     pid_output, self.error = self.pid_controller.compute(target_math_rad + current_body_angle, curr_math_rad)

        #     if(pid_output>0):
        #         direction = 'Q'
        #         weight = pid_output
        #     else:
        #         direction = 'E'
        #         weight = -pid_output

        # # 이미지 픽셀 기반 적 추적 제어
        # elif(self.case == 3):
        #     self.pid_controller.kp = 0.0011
        #     self.pid_controller.ki = 0.0000001
        #     self.pid_controller.kd = 0.0005
        #     self.pid_controller.dt = 0.1
        #     self.pid_controller.ff_gain = 1.0

        #     pid_output, self.error = self.pid_controller.compute(self.target_pixel, self.p_center_x)
            
        #     if self.move_rf == 'A':
        #         pid_output -= self.ff_gain * self.rf_weight
        #     else:
        #         pid_output -= self.ff_gain * self.rf_weight

        #     if(pid_output>0):
        #         direction = 'Q'
        #         weight = pid_output
        #     else:
        #         direction = 'E'
        #         weight = -pid_output


        # 퍼블리시: 현재 & 목표 각도
        self.pub_current_angle.publish(Float32(data=self.current_angle))
        self.pub_target_angle.publish(Float32(data=self.target_angle))

        # 퍼블리시: pitch 제어 명령
        cmd = String()
        cmd.data = f"{direction},{weight:.3f}"
        #self.get_logger().info(f"Publishing /turret_pitch_control → {cmd.data}")
        self.pub_pitch_cmd.publish(cmd)

    def run(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = TurretPitchControlNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
