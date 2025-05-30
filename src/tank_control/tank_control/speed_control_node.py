#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Float32
from tank_package_msg.msg import TankState
from tank_control.pid_controller import PIDVelController
import numpy as np
from geometry_msgs.msg import Point


class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('speed_control_node')

        # QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # velocity PID 파라미터 선언
        for name, default in [
            ('kp_vel', 4.1), ('ki_vel', 0.0), ('kd_vel', 0.5),
            ('dt_vel', 0.1), ('target_speed', 0.0)
        ]:
            self.declare_parameter(name, default)

        kp = self.get_parameter('kp_vel').value
        ki = self.get_parameter('ki_vel').value
        kd = self.get_parameter('kd_vel').value
        dt = self.get_parameter('dt_vel').value
        self.target_speed = self.get_parameter('target_speed').value

        # PIDVelController 생성
        self.pid_vel = PIDVelController(kp=kp, ki=ki, kd=kd, dt=dt)
        self.add_on_set_parameters_callback(self._on_param_change)

        # 구독: 현재 상태(TankState)
        self.sub_state = self.create_subscription(
            TankState, '/tank_state', self.state_callback, qos
        )
        # 퍼블리시: 속도 제어 명령
        self.pub_speed_cmd = self.create_publisher(
            String, '/speed_command', qos
        )

        # 이전 위치 저장용
        self.prev_position = Point()
        self.pub_current_vel = self.create_publisher(Float32, '/speed/current_vel', qos)
        self.pub_target_vel = self.create_publisher(Float32, '/speed/target_vel', qos)
        self.current_vel = 0.0

        self.get_logger().info('SpeedControlNode started')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'kp_vel':      self.pid_vel.kp = p.value
            elif p.name == 'ki_vel':    self.pid_vel.ki = p.value
            elif p.name == 'kd_vel':    self.pid_vel.kd = p.value
            elif p.name == 'dt_vel':    self.pid_vel.dt = p.value
            elif p.name == 'target_speed': self.target_speed = p.value
        return SetParametersResult(successful=True)

    def state_callback(self, msg: TankState):
        # 현재 속도 및 위치, yaw
        current_speed = msg.speed*3.6
        curr_pos = msg.position

        yaw_deg = msg.body_euler.z
        yaw_rad = np.deg2rad(yaw_deg)

        # 이동 방향 기본값
        moving_direction = 1.0
        # 전진/후진 판별
        if self.prev_position is not None:
            dx = curr_pos.x - self.prev_position.x
            dy = curr_pos.y - self.prev_position.y
            v_move = np.array([dx, dy])
            v_forward = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
            md = float(np.sign(np.dot(v_forward, v_move)))
        self.prev_position = curr_pos
        #self.get_logger().info(f' mov_dir: {dx:.4f}, {dy:.4f} , {-yaw_deg}')
        
        if md != 0.0:
            self.current_vel = md * current_speed
        elif current_speed == 0.0:
            self.current_vel = current_speed

        #self.current_vel = current_speed
        self.pub_current_vel.publish(Float32(data=self.current_vel))
        self.pub_target_vel.publish(Float32(data=self.target_speed))
        #self.get_logger().info(f' speed: {self.current_vel:.4f} ')
        #self.get_logger().info(f'current_speed: {current_speed:.4f}')

        # PID 연산 (compute는 반환값 없으므로 control_signal 사용)
        self.pid_vel.compute(self.target_speed, self.current_vel)
        
        # 방향 반영
        ### 차체 속도 제어기(PID)
        # body_vel_pid.compute(shared['tar_tank_vel_kh'], shared['cur_tank_vel_kh'])
        # if body_vel_pid.contorl_signal > 0:
        #     print('wwwwwwwwwwwww {0}'.format(body_vel_pid.contorl_signal))
        #     command['moveWS']['command'] = 'W'
        #     command['moveWS']['weight'] = body_vel_pid.contorl_signal
        # else:
        #     print('sssssssssssss {0}'.format(body_vel_pid.contorl_signal))
        #     command['moveWS']['command'] = 'S'
        #     command['moveWS']['weight'] = -body_vel_pid.contorl_signal
        # 명령 결정


        if self.pid_vel.contorl_signal > 0:
            direction, weight = 'W', self.pid_vel.contorl_signal
        else : 
            direction, weight = 'S', self.pid_vel.contorl_signal

        # 퍼블리시
        cmd = String()
        cmd.data = f"{direction},{float(weight)}"
        self.get_logger().info(f'Publishing /speed_command → {cmd.data}')
        self.pub_speed_cmd.publish(cmd)

    def run(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = SpeedControlNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
