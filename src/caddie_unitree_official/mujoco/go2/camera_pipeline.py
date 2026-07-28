#!/usr/bin/env python3
import math
import numpy as np
import mujoco

class SimulatedCameraPipeline:
    def __init__(self, max_distance=4.0, fov_degrees=60.0):
        self.max_distance = max_distance
        self.fov_rad = math.radians(fov_degrees)

    def track_ball(self, model, data, robot_x, robot_y, robot_yaw):
        """
        Calculates whether the golf ball is within camera FOV and returns true ball coordinates.
        """
        try:
            ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "golf_ball")
            real_ball_x = data.xpos[ball_body_id][0]
            real_ball_y = data.xpos[ball_body_id][1]
            
            dist = np.sqrt((real_ball_x - robot_x)**2 + (real_ball_y - robot_y)**2)
            global_heading = np.arctan2(real_ball_y - robot_y, real_ball_x - robot_x)
            
            angle_err = global_heading - robot_yaw
            angle_err = np.arctan2(np.sin(angle_err), np.cos(angle_err))

            spotted = (dist <= self.max_distance) and (abs(angle_err) <= self.fov_rad)
            return spotted, real_ball_x, real_ball_y, dist, angle_err
        except Exception:
            return False, 0.0, 0.0, 0.0, 0.0