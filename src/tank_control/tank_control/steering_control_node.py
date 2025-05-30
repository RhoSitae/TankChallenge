#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, String, Float32
from tank_package_msg.msg import TankState
import numpy as np
from tank_control.steering_plan import SteeringPlan
from tank_control.pid_controller import PIDDegController
import math

def unity_deg_to_math_rad(angle_deg):
    deg_math = 90.0 - angle_deg
    rad = np.deg2rad(deg_math)
    return (rad + np.pi) % (2*np.pi) - np.pi  # normalize to [-π, π)

class SteeringControlNode(Node):
    def __init__(self):
        super().__init__('steering_control_node')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 파라미터 선언 및 초기값
        for name, default in [
            ('kp', 2.1), ('ki', 0.0), ('kd', 0.05), ('dt', 0.1),
            ('deadzone', 0.0),('target_angle', 0.0), ('case', 0),
            # LAD 튜닝용 파라미터
            ('lad_gain', 0.5), ('min_lad', 5.0), ('max_lad', 12.0),
            # Sine wave case3 파라미터
            ('sin_amplitude', 10.0),  # degrees
            ('sin_period', 5.0),      # seconds
            ('ramp_rate', 15.0)    # degrees per second
        ]:
            self.declare_parameter(name, default)

        # PID 파라미터
        kp    = self.get_parameter('kp').value
        ki    = self.get_parameter('ki').value
        kd    = self.get_parameter('kd').value
        dt    = self.get_parameter('dt').value
        self.deadzone = self.get_parameter('deadzone').value
        self.target_angle = self.get_parameter('target_angle').value
        self.case = self.get_parameter('case').value

        # LAD 파라미터
        lad_gain = self.get_parameter('lad_gain').value
        min_lad  = self.get_parameter('min_lad').value
        max_lad  = self.get_parameter('max_lad').value

        # Sine 파라미터
        self.sin_amplitude = self.get_parameter('sin_amplitude').value
        self.sin_period    = self.get_parameter('sin_period').value
        # 시작 시간 (초)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Ramp 파라미터 초기값
        self.ramp_rate = self.get_parameter('ramp_rate').value

        # PID 컨트롤러 생성
        self.pid_controller = PIDDegController(kp=kp, ki=ki, kd=kd, dt=dt)
        self.add_on_set_parameters_callback(self._on_param_change)

        # Pure Pursuit Steering Plan (LAD 파라미터 반영)
        self.steering_plan = SteeringPlan(
            wheelbase=0.1,
            lad_gain=lad_gain,
            min_lad=min_lad,
            max_lad=max_lad
        )

        # 구독/퍼블리시 설정
        self.sub_state         = self.create_subscription(
            TankState, '/tank_state', self.state_callback, qos
        )
        self.sub_path          = self.create_subscription(
            Marker, '/global_path', self.path_callback, qos
        )
        self.pub_cmd_rate       = self.create_publisher(Float32,  '/steering_cmd',      qos)
        self.pub_command       = self.create_publisher(String,  '/steering_command',      qos)
        self.pub_marker        = self.create_publisher(Marker,  '/steering_debug_markers',qos)
        self.pub_target_angle  = self.create_publisher(Float32, '/steering/target_angle', qos)
        self.pub_current_angle = self.create_publisher(Float32, '/steering/current_angle',qos)
        self.pub_body_case     = self.create_publisher(Float32, '/steering/body_case',qos)
        # 내부 상태 초기화
        self.current_state       = None
        self.current_path_points = None
        self.yaw_deg_pre         = 0.0
        self.yaw_rotation_cnt    = 0
        self.yaw_deg_acc         = 0.0

        self.get_logger().info(f"SteeringControlNode started (LAD gain={lad_gain}, min={min_lad}, max={max_lad})")

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
            elif p.name == 'deadzone':
                self.deadzone = p.value; updated.append('deadzone')
            elif p.name == 'target_angle':
                self.target_angle = p.value; updated.append('target_angle')
            elif p.name == 'lad_gain':
                self.steering_plan.lad_gain = p.value; updated.append('lad_gain')
            elif p.name == 'min_lad':
                self.steering_plan.min_lad = p.value; updated.append('min_lad')
            elif p.name == 'max_lad':
                self.steering_plan.max_lad = p.value; updated.append('max_lad')
            elif p.name == 'case':
                self.case = p.value; updated.append('case')
            elif p.name == 'sin_amplitude':
                self.sin_amplitude = p.value; updated.append('sin_amplitude')
            elif p.name == 'sin_period':
                self.sin_period = p.value; updated.append('sin_period')
            elif p.name == 'ramp_rate':
                self.ramp_rate = p.value; updated.append('ramp_rate')

        #if updated:
            #self.get_logger().info(f"Parameters updated: {', '.join(updated)}")
        return SetParametersResult(successful=True)

    def state_callback(self, msg: TankState):
        self.current_state = msg
        self.try_control()

    def path_callback(self, msg: Marker):
        self.current_path_points = [(p.x, p.y, p.z) for p in msg.points]
        self.try_control()

    def try_control(self):

        if self.current_state is None:
            return
        if(self.case == 2): # case2
            if self.current_path_points is None:
                return

        # 현재 위치·yaw 계산
        x, y, z = (
            self.current_state.position.x,
            self.current_state.position.y,
            self.current_state.position.z
        )
        yaw_deg = self.current_state.body_euler.z +360.0

        # # yaw 누적 처리
        # if   (self.yaw_deg_pre - yaw_deg) > 270:
        #     self.yaw_rotation_cnt += 1
        # elif (self.yaw_deg_pre - yaw_deg) < -270:
        #     self.yaw_rotation_cnt -= 1
        # self.yaw_deg_acc  = yaw_deg + self.yaw_rotation_cnt * 360
        # self.yaw_deg_pre  = yaw_deg
        yaw_rad = np.deg2rad(yaw_deg)

        steer_angle_deg = 0.0
        direction = ""
        weight = 0.0
        if(self.case == 0): # case 0
            
            # 토픽 퍼블리시
            self.pub_target_angle.publish(Float32(data=steer_angle_deg))
            self.pub_current_angle.publish(Float32(data=yaw_deg))

        elif(self.case == 1): # case 1 # 타겟 각도 지정
            self.pid_controller.kp = 2.1
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.05
            self.pid_controller.dt = 0.1

            steer_angle_deg = self.target_angle
            steer_rad_target  = np.deg2rad(steer_angle_deg)
            output, error = self.pid_controller.compute(steer_rad_target, yaw_rad)
            direction = 'D' if output < 0 else 'A'
            weight    = float(output)

            # 토픽 퍼블리시
            self.pub_target_angle.publish(Float32(data=steer_angle_deg))
            self.pub_current_angle.publish(Float32(data=yaw_deg))

        elif(self.case == 2): # case 2 # pure pursuit 자율주행
            self.pid_controller.kp = 4.1
            self.pid_controller.ki = 0.0
            self.pid_controller.kd = 0.5
            self.pid_controller.dt = 0.1


            # SteeringPlan 업데이트
            self.steering_plan.update_vehicle_state(x, y, z, self.current_state.speed, yaw_rad)
            waypoints = np.array(self.current_path_points)
            self.steering_plan.waypoints = waypoints

            # Pure Pursuit → PID → 명령 퍼블리시
            lad             = self.steering_plan.lad()
            target_wp       = self.steering_plan.find_target_waypoint(waypoints)
            steer_angle_deg = self.steering_plan.pure_pursuit(target_wp, lad)
            steer_rad_target  = steer_angle_deg
            target_math_rad = (steer_rad_target + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화
            output, error = self.pid_controller.compute(0.0, target_math_rad)
            direction = 'D' if output > 0 else 'A'
            weight    = float(output)

            # 토픽 퍼블리시
            self.pub_target_angle.publish(Float32(data=0.0))
            self.pub_current_angle.publish(Float32(data=target_math_rad))

            # 디버그 마커
            nearest_idx, nearest_wp = self.steering_plan.find_nearest_waypoint([x, y, z], waypoints)
            for idx, wp in [('nearest', nearest_wp), ('target', target_wp)]:
                m = Marker()
                m.header.frame_id    = 'map'
                m.header.stamp       = self.get_clock().now().to_msg()
                m.ns                 = 'debug_markers'
                m.id                 = 0 if idx == 'nearest' else 1
                m.type               = Marker.SPHERE
                m.action             = Marker.ADD
                m.pose.position      = Point(x=wp[0], y=wp[1], z=wp[2])
                m.pose.orientation.w = 1.0
                m.scale              = Vector3(x=5.5, y=0.5, z=0.5) if idx == 'nearest' else Vector3(x=0.5, y=0.5, z=5.5)
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) if idx == 'nearest' \
                          else ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                self.pub_marker.publish(m)
        elif self.case == 3:
            # Sine wave 타겟 각도
            now = self.get_clock().now().nanoseconds / 1e9
            t = now - self.start_time
            # 주기(period) 만큼 반복
            steer_angle_deg = self.sin_amplitude * math.sin(2 * math.pi * t / self.sin_period)
            steer_rad_target = steer_angle_deg
            output, error = self.pid_controller.compute(steer_rad_target, yaw_rad)
            direction = 'A' if output > 0 else 'D'
            weight = float(output)

            # 퍼블리시
            self.pub_target_angle.publish(Float32(data=steer_angle_deg))
            self.pub_current_angle.publish(Float32(data=yaw_rad))

        elif self.case == 4:
   
            # Ramp: 시작 각도(self.target_angle)에서 초당 self.ramp_rate 만큼 선형 증가
            now = self.get_clock().now().nanoseconds / 1e9
            t = now - self.start_time
            steer_angle_deg = self.target_angle + self.ramp_rate * t
            steer_rad_target = steer_angle_deg
            # PID 제어
            output, error = self.pid_controller.compute(steer_rad_target, yaw_rad)
            direction = 'A' if output > 0 else 'D'
            weight = float(output)

            # 퍼블리시
            self.pub_target_angle.publish(Float32(data=steer_rad_target))
            self.pub_current_angle.publish(Float32(data=yaw_rad))

        # Command string & 퍼블리시
        cmd = String()
        cmd.data = f"{direction},{weight:.3f}"
        self.pub_cmd_rate.publish(Float32(data=weight))
        self.pub_body_case.publish(Float32(data=float(self.case)))
        self.pub_command.publish(cmd)
        

        # steer_angle_deg = self.deadzone
        # steer_rad_target  = np.deg2rad(steer_angle_deg)
        # target_math_rad = (steer_rad_target + np.pi) % (2*np.pi) - np.pi  # [-π, π) 범위로 정규화
        # pure pursuit 기반 차체 조향 제어
        # output, error = self.pid_controller.compute(0.0, target_math_rad)

        # global 좌표계 오일러 target 각도 기반 차체 조향 제어
        


        


        
        # cmd = String()
        # cmd.data = f"{direction},{weight:.3f}"

        # self.pub_cmd_rate.publish(Float32(data=weight))
        # self.pub_command.publish(cmd)

        

        
    def run(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = SteeringControlNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
