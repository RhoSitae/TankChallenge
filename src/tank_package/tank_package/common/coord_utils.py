# tank_package/common/coord_utils.py
import math
from geometry_msgs.msg import Point, Quaternion
from tf_transformations import quaternion_from_euler

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """Roll, Pitch, Yaw을 쿼터니언으로 변환 (부호 반전 포함)"""
    qx, qy, qz, qw = quaternion_from_euler(-roll, pitch, -yaw)  # 부호 반전하여 쿼터니언 변환
    return qx, qy, qz, qw

def ros_to_unity_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """ROS 좌표계의 Euler Angles (Roll, Pitch, Yaw) -> Unity 좌표계의 Euler Angles 변환 (부호 반전 포함)"""
    return -roll, pitch, -yaw

def unity_to_ros_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Unity 좌표계의 Euler Angles (Roll, Pitch, Yaw) -> ROS 좌표계의 Euler Angles 변환 (부호 반전 포함)"""
    return -roll, pitch, -yaw

def unity_point_to_ros(u: dict) -> Point:
    """Unity 좌표계의 점 (x, z, y) -> ROS 좌표계의 점 (X, Y, Z) 변환 (부호 반전 포함)"""
    p = Point()
    p.x = float(u.get("z", 0.0))  # Unity z -> ROS x
    p.y = -float(u.get("x", 0.0))  # Unity x -> ROS y (부호 반전)
    p.z = float(u.get("y", 0.0))   # Unity y -> ROS z
    return p

def ros_point_to_unity(r: dict) -> Point:
    """ROS 좌표계의 점 (X, Y, Z) -> Unity 좌표계의 점 (x, z, y) 변환 (부호 반전 포함)"""
    p = Point()
    p.x = -float(r.get("y", 0.0))  # ROS y -> Unity x (부호 반전)
    p.y = float(r.get("z", 0.0))   # ROS z -> Unity y
    p.z = float(r.get("x", 0.0))   # ROS x -> Unity z
    return p
