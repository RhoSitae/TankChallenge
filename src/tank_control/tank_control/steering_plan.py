import numpy as np

class SteeringPlan:
    def __init__(self, waypoints=None, wheelbase=0.001, lad_gain=1.5, min_lad=20.0, max_lad=50.0):
        self.veh_x = None
        self.veh_y = None
        self.veh_z = None
        self.veh_speed = None
        self.veh_heading = None
        self.waypoints = waypoints
        self.wheelbase = wheelbase
        self.lad_gain = lad_gain
        self.min_lad = min_lad
        self.max_lad = max_lad

    def update_vehicle_state(self, x, y, z, speed, yaw_math_rad):
        self.veh_x = x
        self.veh_y = y
        self.veh_z = z
        self.veh_speed = speed
        self.veh_heading = yaw_math_rad

    def lad(self):
        lad_val = self.lad_gain * self.veh_speed + self.min_lad
        lad_val = np.clip(lad_val, self.min_lad, self.max_lad)
        return lad_val

    def find_nearest_waypoint(self, veh_pos, rddf_map):
        veh_pos = np.array(veh_pos).reshape(1, 3)
        distances = np.linalg.norm(rddf_map - veh_pos, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_point = rddf_map[nearest_idx]
        return nearest_idx, nearest_point

    def find_target_waypoint(self, rddf_map):
        veh_pos = np.array([self.veh_x, self.veh_y, self.veh_z])
        nearest_idx, nearest_point = self.find_nearest_waypoint(veh_pos, rddf_map)

        lad = self.lad()
        distances = np.linalg.norm(rddf_map - veh_pos.reshape(1, 3), axis=1)

        target_wp = None
        for i in range(nearest_idx, len(rddf_map)):
            if distances[i] >= lad:
                target_wp = rddf_map[i]
                break

        if target_wp is None:
            target_wp = rddf_map[-1]

        return target_wp

    def pure_pursuit(self, target_point, lad):
        veh_pos_2d = np.array([self.veh_x, self.veh_y])
        target_point_2d = np.array([target_point[0], target_point[1]])

        delta = target_point_2d - veh_pos_2d

        angle_to_target = np.arctan2(delta[1], delta[0])
        alpha = angle_to_target - self.veh_heading
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        steer_angle_rad = np.arctan2(2 * self.wheelbase * np.sin(alpha), lad)
        steer_angle_deg = np.rad2deg(steer_angle_rad)
        return steer_angle_deg
