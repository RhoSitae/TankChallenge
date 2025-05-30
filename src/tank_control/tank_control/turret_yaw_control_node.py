#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Float32
from tank_package_msg.msg import TankState
from tank_control.pid_controller import PIDDegController
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from functools import partial

class TurretYawControlNode(Node):
    def __init__(self):
        super().__init__('turret_yaw_control_node')

        # QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 파라미터 선언 (PID + 목표각 + FF gain)
        for name, default in [

            # 수동 제어기 튜닝
            # 사용자
            # ('kp_0', 1), ('ki_0', 0), ('kd_0', 0),
            # ('dt_0', 0.1), ('target_angle_0', 960.0),
            # ('ff_gain_0', 1.0),('case_0', 0)

            # 각도 기반 제어기 튜닝
            # 사용자가 입력한 절대 각도를 추종함
            ('kp', 4.1), ('ki', 0.0), ('kd', 0.05),
            ('dt', 0.1), ('target_angle', 0.0),('target_pixel', 960.0),
            ('ff_gain', 1.0), ('case', 0), ('tracking_assi_p', 0.0)

            # 이미지 픽셀 기반 제어기 튜닝
            # ('kp', 0.0011), ('ki', 0.0000001), ('kd', 0.0005),
            # ('dt', 0.1), ('target_angle', 960.0),
            # ('ff_gain', 1.0)



            # if( case == 0):
            #     튜닝값 1
            # elif (case == 1):
            #     튜닝값 2
            # elif (case == 2):
            #     튜닝값 3
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
        self.track_p = self.get_parameter('tracking_assi_p').value

        # PID 컨트롤러 생성
        self.pid_controller = PIDDegController(kp=kp, ki=ki, kd=kd, dt=dt)

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self._on_param_change)

        # 내부 상태 초기화
        self.move_ad     = 'A'
        self.ad_weight   = 0.0
        self.current_angle = 0.0
        self.error = 0.0
        self.p_center_x = 0.0
        self.p_center_y = 0.0
        self.body_case = 0
        # 구독: TankState
        self.create_subscription(
            TankState, '/tank_state',
            self.state_callback,
            qos
        )
        # 구독: steering_command
        self.create_subscription(
            String, '/steering_command',
            partial(self.command_callback, 'moveAD'),
            qos
        )
        self.create_subscription(
            PointStamped, '/yolo_detector/center_point',
            self.center_point_callback,
            qos
        )
        self.create_subscription(
            Float32, '/steering/body_case',
            self.body_case_callback,
            qos
        )
        self.create_subscription(
            Float32, '/speed/current_vel',
            self.body_vel,
            qos
        )




        # 퍼블리시: turret yaw 명령
        self.pub_yawing_cmd    = self.create_publisher(String,  '/turret_yaw_control',     qos)
        self.pub_current_angle = self.create_publisher(Float32, '/turret_yaw/current_angle', qos)
        self.pub_target_angle  = self.create_publisher(Float32, '/turret_yaw/target_angle', qos)
        self.pub_image_center  = self.create_publisher(Float32, '/turret_yaw/image_center',qos)
        self.pub_target_pixel  = self.create_publisher(Float32, '/turret_yaw/target_pixel',qos)

        self.get_logger().info(f'TurretYawControlNode started')

    def _on_param_change(self, params):
        updated = []
        for p in params:
            # if p.name == 'case':
            #     if p.value == 0:

            #     elif p.value == 1:

            #     elif p.value == 2:

            #     self.pid_controller.kp = p.
            #     self.pid_controller.ki = p.value; updated.append('ki')
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
            elif p.name == 'tracking_assi_p':
                self.track_p = p.value; updated.append('tracking_assi_p')
                    

        #if updated:
            #self.get_logger().info(f'Parameters updated: {", ".join(updated)}')
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
        self.move_ad   = direction
        self.ad_weight = weight
        # self.get_logger().info(f"[{topic_key}] ← {direction},{weight:.3f}")
    def center_point_callback(self, msg: PointStamped):
        self.p_center_x = msg.point.x
        self.p_center_y = msg.point.y

    def body_case_callback(self, msg: Float32):
        self.body_case = int(msg.data)

    def body_vel(self, msg: Float32):
        self.body_vel = float(msg.data)

    def state_callback(self, msg: TankState):
        """ TankState 수신 시 PID + FF 계산하여 turret yaw 명령 퍼블리시 """
        direction = ''
        weight = 0.0
        a_weight = 0.0

        # 현재 body 각도 (deg)
        current_body_angle = np.deg2rad(msg.body_euler.z + 360)

        # 현재 turret 각도 (deg)
        self.current_angle = msg.turret_euler.z + 360
        yaw_rad_current    = np.deg2rad(self.current_angle)
        yaw_rad_target     = np.deg2rad(self.target_angle)
        curr_math_rad = (yaw_rad_current + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화
        target_math_rad = (yaw_rad_target + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화
        
        # 현재 이미지 중심 픽셀
        self.pub_image_center.publish(Float32(data=self.p_center_x))
        self.pub_target_pixel.publish(Float32(data=self.target_pixel))
        
        # if self.case == 0:
        #     pass
        # elif self.case == 1:
        #     # 절대 각도기반 포탑 제어 + Feed-forward 보상

        #     # 전향 보상 텀
        #     if self.move_ad == 'A':
        #         pid_output += self.ff_gain * self.ad_weight
        #     else:
        #         pid_output += self.ff_gain * self.ad_weight

        #     if(pid_output>0):
        #         direction = 'Q'
        #         weight = pid_output
        #     else:
        #         direction = 'E'
        #         weight = -pid_output

        # elif self.case == 2:
        #     # 절대 각도기반 포탑 제어 + Feed-forward 보상

        #     # 전향 보상 텀
        #     if self.move_ad == 'A':
        #         pid_output -= self.ff_gain * self.ad_weight
        #     else:
        #         pid_output -= self.ff_gain * self.ad_weight

        #     if(pid_output>0):
        #         direction = 'Q'
        #         weight = pid_output
        #     else:
        #         direction = 'E'
        #         weight = -pid_output

        # 절대 각도 기반 제어기 + 전향보상
        

        if(self.case == 0):
            pass

        # global 오일러각 target 각도 추종 + 차체 조향에 따른 전향제어
        elif(self.case == 1): 
            self.pid_controller.kp = 4.1
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.05
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 1.0

            pid_output, self.error = self.pid_controller.compute(target_math_rad, curr_math_rad)
            
            if self.move_ad == 'A':
                if(self.body_case == 1): # target a gle 지정
                    self.get_logger().info(f'aaaaaaaaaaaa')
                    pid_output -= self.ff_gain * self.ad_weight
                elif(self.body_case == 2): # pure pursuit
                    self.get_logger().info(f'bbbbbbbbbbbbbbb')
                    pid_output += self.ff_gain * self.ad_weight
            else:
                if(self.body_case == 1): # target angle 지
                    self.get_logger().info(f'aaaaaaaaaaaa')
                    pid_output -= self.ff_gain * self.ad_weight
                elif(self.body_case == 2): # pure pursuit
                    self.get_logger().info(f'bbbbbbbbbbbbbbb')
                    pid_output += self.ff_gain * self.ad_weight

            if(pid_output>0):
                direction = 'Q'
                weight = pid_output
            else:
                direction = 'E'
                weight = -pid_output

        # 차체 기준 전방 오일러각 target 각도 추종  + 전향제어 제
        elif(self.case == 2):
            self.pid_controller.kp = 4.1
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.05
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 0.0

            pid_output, self.error = self.pid_controller.compute(target_math_rad + current_body_angle, curr_math_rad)
            
            # if self.move_ad == 'A':
            #     pid_output -= self.ff_gain * self.ad_weight
            # else:
            #     pid_output -= self.ff_gain * self.ad_weight

            if(pid_output>0):
                direction = 'Q'
                weight = pid_output
            else:
                direction = 'E'
                weight = -pid_output

        # 이미지 픽셀 기반 적 추적 제어
        elif(self.case == 3):
            self.pid_controller.kp = 0.0011
            self.pid_controller.ki = 0.0#0.0000001
            self.pid_controller.kd = 0.0005
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 1.0

            pid_output, self.error = self.pid_controller.compute(self.target_pixel, self.p_center_x)
            
            if self.move_ad == 'A':
                pid_output -= self.ff_gain * self.ad_weight
            else:
                pid_output -= self.ff_gain * self.ad_weight

            if(pid_output>0):
                direction = 'Q'
                weight = pid_output
            else:
                direction = 'E'
                weight = -pid_output
        
        # 이미지 픽셀 기반 적 추적 제어
        elif(self.case == 4):
            self.pid_controller.kp = 0.0011
            self.pid_controller.ki = 0.0#0.0000001
            self.pid_controller.kd = 0.0005
            self.pid_controller.dt = 0.1
            self.pid_controller.ff_gain = 0.11

            pid_output, self.error = self.pid_controller.compute(self.target_pixel, self.p_center_x)
            
            if self.move_ad == 'A':
                pid_output -= self.ff_gain * self.ad_weight
            else:
                pid_output -= self.ff_gain * self.ad_weight

            # self.get_logger().info(f'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa {np.rad2deg(self.body_vel)} {temp}')
            # self.get_logger().info(f'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa {np.rad2deg(current_body_angle)}')
            # self.get_logger().info(f'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb {np.rad2deg(yaw_rad_current)}')
            # self.get_logger().info(f'ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff {360-(+np.rad2deg(current_body_angle)+360 - np.rad2deg(yaw_rad_current)-360)%360}')
            turret_del_rad = np.deg2rad(360-(+np.rad2deg(current_body_angle)+360 - np.rad2deg(yaw_rad_current)-360)%360)


            if(turret_del_rad<np.pi):
                pid_output += self.body_vel*3.6*np.sin(turret_del_rad)/msg.distance*self.track_p # 영상 표적 추적 안정화
            else:
                pid_output -= self.body_vel*3.6*np.sin(turret_del_rad)/msg.distance*self.track_p # 영상 표적 추적 안정화

            if(pid_output>0):
                direction = 'Q'
                weight = pid_output
            else:
                direction = 'E'
                weight = -pid_output
        
            


        
        

        # 퍼블리시: 현재 & 목표 각도
        self.pub_current_angle.publish(Float32(data=self.current_angle))
        self.pub_target_angle.publish(Float32(data=self.target_angle))

        # 퍼블리시: yaw 제어 명령
        cmd = String()
        cmd.data = f"{direction},{weight:.3f}"
        #self.get_logger().info(f"Publishing /turret_yaw_control → {cmd.data}")
        self.pub_yawing_cmd.publish(cmd)

    def run(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = TurretYawControlNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
