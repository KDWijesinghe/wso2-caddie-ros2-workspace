#!/usr/bin/env python3
import math
import time
import numpy as np
import mujoco

class SimulatedCameraPipeline:
    def __init__(self, max_distance=8.0, fov_degrees=85.0, memory_timeout=1.0):
        # 1. පෙනෙන දුර මීටර් 8 ක් දක්වා වැඩි කළා
        self.max_distance = max_distance
        
        # 2. FOV එක අංශක 85 (දෙපැත්තටම අංශක 170ක Peripheral Vision එකක්) දක්වා වැඩි කළා
        self.fov_rad = math.radians(fov_degrees)
        
        # 3. Object Permanence (මතක තබා ගැනීමේ කාලය) - තත්පර 1 යි
        self.memory_timeout = memory_timeout
        self.last_seen_time = 0.0
        self.last_known_x = 0.0
        self.last_known_y = 0.0

    def track_ball(self, model, data, robot_x, robot_y, robot_yaw):
        """
        Calculates whether the golf ball is within camera FOV and returns true ball coordinates.
        Includes a memory buffer to prevent instant tracking loss.
        """
        try:
            ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "golf_ball")
            real_ball_x = data.xpos[ball_body_id][0]
            real_ball_y = data.xpos[ball_body_id][1]
            
            dist = np.sqrt((real_ball_x - robot_x)**2 + (real_ball_y - robot_y)**2)
            global_heading = np.arctan2(real_ball_y - robot_y, real_ball_x - robot_x)
            
            angle_err = global_heading - robot_yaw
            angle_err = np.arctan2(np.sin(angle_err), np.cos(angle_err))

            # ඇත්තටම බෝලය කැමරාවට පේනවාද කියා බැලීම
            in_vision = (dist <= self.max_distance) and (abs(angle_err) <= self.fov_rad)
            
            current_time = time.time()
            
            if in_vision:
                # පේනවා නම් අලුත්ම තොරතුරු මතකයට (Memory) දාගන්නවා
                self.last_seen_time = current_time
                self.last_known_x = real_ball_x
                self.last_known_y = real_ball_y
                spotted = True
            else:
                # පේන්නේ නැති වුණත්, අන්තිමට දැකලා තත්පර 1ක් (memory_timeout) ගතවෙලා නැත්නම්, 
                # බල්ලා ඒ අන්තිමට දැක්ක තැන මතක තියාගෙන Track කරනවා.
                if (current_time - self.last_seen_time) < self.memory_timeout:
                    spotted = True
                else:
                    spotted = False
            
            # Return කරන්නේ අන්තිමට හරියටම දැක්ක (X, Y) ඛණ්ඩාංකයි
            return spotted, self.last_known_x, self.last_known_y, dist, angle_err
            
        except Exception:
            return False, 0.0, 0.0, 0.0, 0.0