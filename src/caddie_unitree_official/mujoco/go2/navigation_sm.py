# #!/usr/bin/env python3
# import numpy as np
# import time

# class NavigationStateMachine:
#     def __init__(self, logger):
#         self.logger = logger
#         self.state = 'MANUAL'
#         self.target_x = 0.0
#         self.target_y = 0.0
#         self.search_start_time = 0.0

#     def compute_target_velocity(self, robot_x, robot_y, robot_yaw, ball_is_spotted):
#         """
#         Evaluates the current state and returns the raw target velocity [vx, vy, wz].
#         """
#         target_velocity = [0.0, 0.0, 0.0]

#         if self.state == 'WAYPOINT_APPROACH':
#             dx = self.target_x - robot_x
#             dy = self.target_y - robot_y
#             distance = np.sqrt(dx**2 + dy**2)
            
#             global_heading = np.arctan2(dy, dx)
#             local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))

#             if distance < 0.30:
#                 self.logger.info("📍 Arrived at target area. Entering STANDBY_AT_WAYPOINT...")
#                 self.state = 'STANDBY_AT_WAYPOINT'
#             else:
#                 if abs(local_angle) > 0.45:
#                     target_velocity = [0.0, 0.0, np.clip(0.6 * local_angle, -0.45, 0.45)]
#                 else:
#                     target_velocity = [
#                         min(0.45, 0.6 * distance) * np.cos(local_angle),
#                         min(0.15, 0.3 * distance * np.sin(local_angle)),
#                         np.clip(0.5 * local_angle, -0.30, 0.30)
#                     ]

#         elif self.state == 'LOCAL_SEARCH':
#             elapsed_time = time.time() - self.search_start_time
#             spiral_radius = min(2.0, 0.05 * elapsed_time)
#             target_velocity = [0.12, min(0.15, spiral_radius * 0.12), 0.35]

#         elif self.state == 'FINAL_INTERCEPT':
#             dx = self.target_x - robot_x
#             dy = self.target_y - robot_y
#             distance = np.sqrt(dx**2 + dy**2)

#             if distance < 0.20:
#                 self.logger.info("⚽ BALL SECURED! Returning to MANUAL state.")
#                 self.state = 'MANUAL'
#             else:
#                 global_heading = np.arctan2(dy, dx)
#                 local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))

#                 if abs(local_angle) > 0.35:
#                     vx, vy = 0.0, 0.0
#                     wz = np.clip(0.4 * local_angle, -0.25, 0.25)
#                 else:
#                     vx = min(0.40, 0.6 * distance) * max(0.1, np.cos(local_angle))
#                     vy = min(0.12, 0.25 * distance * np.sin(local_angle))
#                     wz = np.clip(0.3 * local_angle, -0.15, 0.15)
#                 target_velocity = [vx, vy, wz]

#         elif self.state == 'RETURNING_HOME':
#             dx = 0.0 - robot_x
#             dy = 0.0 - robot_y
#             dist_home = np.sqrt(dx**2 + dy**2)
#             local_angle_home = np.arctan2(np.sin(np.arctan2(dy, dx) - robot_yaw), np.cos(np.arctan2(dy, dx) - robot_yaw))

#             if dist_home < 0.22:
#                 self.logger.info("Arrived safely back at home base.")
#                 self.state = 'MANUAL'
#             else:
#                 target_velocity = [
#                     min(0.45, 0.6 * dist_home) * max(0.0, np.cos(local_angle_home)),
#                     min(0.15, 0.3 * dist_home * np.sin(local_angle_home)),
#                     np.clip(0.5 * local_angle_home, -0.35, 0.35)
#                 ]

#         return target_velocity

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
        """
        බෝලය Track කිරීම, බෝලය ළඟට යෑම සහ ආපහු බෝලය අරන් එන වේගයන් ගණනය කිරීම.
        """
        target_velocity = [0.0, 0.0, 0.0]

        # 1. TRACKING_VISUAL: බෝලය පෙරලෙනකොට බල්ලා නොනැවතී බෝලය දිහාම බලා සිටීම
        if self.state == 'TRACKING_VISUAL':
            if ball_is_spotted:
                global_heading = np.arctan2(ball_y - robot_y, ball_x - robot_x)
                local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))
                if abs(local_angle) > 0.15:
                    target_velocity = [0.0, 0.0, np.clip(0.5 * local_angle, -0.3, 0.3)]

        # 2. GOTO_BALL: කෙලින්ම බෝලය තියෙන තැනට කකුල් උස්සලා ගමන් කිරීම
        elif self.state == 'GOTO_BALL':
            dx = self.target_x - robot_x
            dy = self.target_y - robot_y
            distance = np.sqrt(dx**2 + dy**2)

            if distance < 0.18: # බෝලය ළඟටම ආවාම Grab State එකට යෑම
                self.logger.info("⚽ බෝලය ළඟට ආවා! බෝලය කටින් Grab කිරීමට සූදානම්...")
                self.state = 'GRAB_BALL'
            else:
                global_heading = np.arctan2(dy, dx)
                local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))

                if abs(local_angle) > 0.35: # මුලින්ම බෝලය දෙසට කෙලින්ම rotate වීම
                    target_velocity = [0.0, 0.0, np.clip(0.4 * local_angle, -0.25, 0.25)]
                else: # ඉන්පසු කෙලින්ම බෝලය දෙසට ඇවිදීම
                    vx = min(0.40, 0.6 * distance) * max(0.1, np.cos(local_angle))
                    vy = min(0.12, 0.25 * distance * np.sin(local_angle))
                    wz = np.clip(0.3 * local_angle, -0.15, 0.15)
                    target_velocity = [vx, vy, wz]

        # 3. RETURN_WITH_BALL: බෝලයත් අරගෙන ආපහු හිටපු තැනට (0,0) පැමිණීම
        elif self.state == 'RETURN_WITH_BALL':
            dx_home = 0.0 - robot_x
            dy_home = 0.0 - robot_y
            dist_home = np.sqrt(dx_home**2 + dy_home**2)
            
            if dist_home < 0.25:
                self.logger.info("🏠 බෝලයත් අරන් සාර්ථකව Home (0,0) එකට ආවා! බෝලය අතහැරියා.")
                self.ball_grabbed = False  # බෝලය අතහැරීම
                self.state = 'MANUAL'
            else:
                global_heading = np.arctan2(dy_home, dx_home)
                local_angle = np.arctan2(np.sin(global_heading - robot_yaw), np.cos(global_heading - robot_yaw))
                
                vx = min(0.40, 0.6 * dist_home) * max(0.1, np.cos(local_angle))
                vy = min(0.12, 0.25 * dist_home * np.sin(local_angle))
                wz = np.clip(0.4 * local_angle, -0.3, 0.3)
                target_velocity = [vx, vy, wz]

        return target_velocity