#!/usr/bin/env python3
import numpy as np
import time

class NavigationStateMachine:
    def __init__(self, logger):
        self.logger = logger
        self.state = 'MANUAL'
        self.target_x = 0.0
        self.target_y = 0.0
        self.ball_grabbed = False

    def compute_target_velocity(self, robot_x, robot_y, robot_yaw, ball_is_spotted, ball_x, ball_y):
        target_velocity = [0.0, 0.0, 0.0]

        # 1 & 2. TRACKING_VISUAL (පස්සෙන් පන්නන) සහ GOTO_BALL (නැවතුණාම යන) අවස්ථා
        if self.state in ['TRACKING_VISUAL', 'GOTO_BALL']:
            if ball_is_spotted:
                self.target_x = ball_x
                self.target_y = ball_y

            dx = self.target_x - robot_x
            dy = self.target_y - robot_y
            distance = np.sqrt(dx**2 + dy**2)

            if self.state == 'GOTO_BALL' and distance < 0.18: 
                self.logger.info("⚽ බෝලය ළඟට ආවා! Grab කරනවා...")
                self.state = 'GRAB_BALL'
                return [0.0, 0.0, 0.0]

            if not ball_is_spotted and distance < 0.3:
                self.logger.info("👀 බෝලය නැති වුණා! වටේ කැරකිලා හොයනවා... (Scanning)")
                return [0.0, 0.0, 0.6]

            global_heading = np.arctan2(dy, dx)
            local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))

            # හැරෙන වේගය (Turn Speed)
            wz = np.clip(1.5 * local_angle, -0.8, 0.8)

            # 🚀 SMOOTH CHASE LOGIC (මෙතන තමයි වෙනස් වුණේ) 🚀
            if self.state == 'TRACKING_VISUAL':
                base_vx = min(0.70, 0.8 * distance)  # චේස් කරද්දී උපරිම වේගය (0.7m/s)
            else:
                base_vx = min(0.40, 0.6 * distance)  # බෝලය නැවතුණාම සාමාන්‍ය වේගයෙන් යනවා

            # අංශක 90 ට වඩා පැත්තට ගියොත් විතරක් නවතිනවා (max(0.0, cos)). 
            # නැත්නම් හැරෙන ගමන්ම Curve එකට ඉස්සරහට දුවනවා!
            vx = base_vx * max(0.0, np.cos(local_angle))

            target_velocity = [vx, 0.0, wz]

        # 3. RETURN_WITH_BALL (බෝලය අරන් ගෙදර ඒම)
        elif self.state == 'RETURN_WITH_BALL':
            dx_home = 0.0 - robot_x
            dy_home = 0.0 - robot_y
            dist_home = np.sqrt(dx_home**2 + dy_home**2)
            
            if dist_home < 0.25:
                self.logger.info("🏠 බෝලයත් අරන් Home (0,0) එකට ආවා! වැඩේ සාර්ථකයි.")
                self.ball_grabbed = False 
                self.state = 'MANUAL'
            else:
                global_heading = np.arctan2(dy_home, dx_home)
                local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))
                
                wz = np.clip(-0.5 * local_angle, -0.6, 0.6)
                
                # ගෙදර එද්දිත් හැරෙන ගමන්ම එන්න හදලා තියෙන්නේ
                base_vx = min(0.40, 0.6 * dist_home)
                vx = base_vx * max(0.0, np.cos(local_angle))
                
                target_velocity = [vx, 0.0, wz]

        return target_velocity