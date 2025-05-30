import numpy as np


class PIDVelController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0
        
        self.control_signal = 0.0
        
        
    def compute(self, target, current):
        # 현재 오차 계산

        error = target - current

        # 적분항 누적
        self.integral += error * self.dt
        
        # 미분항 계산
        d_error = (error - self.prev_error) / self.dt
        
        # 이전 오차 업데이트
        self.prev_error = error
        
        # PID 제어량
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * d_error
        )
        # 리미터 설정
        if (output > 1):
            output = 1
        if (output < -1):
            output = -1
        # print('controller output: ', output)
        
        
        self.contorl_signal = output
        return output
        
    def update_gains(self, kp=None, ki=None, kd=None, dt=None): 
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if dt is not None:
            self.dt = dt
            
            
class PIDDegController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.0, kffp=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.kffp = kffp

        self.integral = 0.0
        self.prev_error = 0.0
        
        self.control_signal = 0.0
        
        
    def compute(self, target, current):
        # 현재 오차 계산
        error = (target - current) # + 180) % 360 - 180
        debug_error = error
        
        if(error<-np.pi):
            error+=np.pi*2
        if(error>np.pi):
            error-=np.pi*2

        # if error>np.pi:
        #     error -= 2*np.pi
            
        # 적분항 누적
        self.integral += error * self.dt
        
        # 미분항 계산
        d_error = (error - self.prev_error) / self.dt
        
        # 이전 오차 업데이트
        self.prev_error = error
        
        # PID 제어량
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * d_error
        )
        
        # 리미터 설정
        if (output > 1):
            output = 1
        if (output < -1):
            output = -1
        
        self.contorl_signal = output
        return output, error

class PIDDegController_non_limit:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.0, kffp=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.kffp = kffp

        self.integral = 0.0
        self.prev_error = 0.0
        
        self.control_signal = 0.0
        
        
    def compute(self, target, current):
        # 현재 오차 계산
        error = (target - current) # + 180) % 360 - 180
        debug_error = error
        
        if(error<-np.pi):
            error+=np.pi*2
        if(error>np.pi):
            error-=np.pi*2

        # if error>np.pi:
        #     error -= 2*np.pi
            
        # 적분항 누적
        self.integral += error * self.dt
        
        # 미분항 계산
        d_error = (error - self.prev_error) / self.dt
        
        # 이전 오차 업데이트
        self.prev_error = error
        
        # PID 제어량
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * d_error
        )
        
        # 리미터 설정
        # if (output > 1):
        #     output = 1
        # if (output < -1):
        #     output = -1
        
        self.contorl_signal = output
        return output, error

    
    
    def update_gains(self, kp=None, ki=None, kd=None, dt=None, kffp=None): 
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if dt is not None:
            self.dt = dt
        if kffp is not None:
            self.kffp = kffp
            