# #!/usr/bin/env python3
# import mujoco
# import mujoco.viewer
# import time
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from std_msgs.msg import String
# import math

# class Go2MujocoWalkBridge(Node):
#     def __init__(self):
#         super().__init__('caddie_mujoco_bridge')
        
#         print("Creating Dynamic Golf Environment in MuJoCo...")
        
#         # --- ⛳ LOAD DYNAMIC GOLF SCENE FROM XML ⛳ ---
#         import os
#         xml_path = os.path.join(os.path.dirname(__file__), 'dynamic_golf_scene.xml')
#         self.model = mujoco.MjModel.from_xml_path(xml_path)
#         self.data = mujoco.MjData(self.model)
        
#         # Stance configurations
#         self.stand_targets = {
#             'FR_hip': 0.0, 'FR_thigh': 0.6, 'FR_calf': -1.2,
#             'FL_hip': 0.0, 'FL_thigh': 0.6, 'FL_calf': -1.2,
#             'RR_hip': 0.0, 'RR_thigh': 0.6, 'RR_calf': -1.2,
#             'RL_hip': 0.0, 'RL_thigh': 0.6, 'RL_calf': -1.2
#         }
        
#         self.kp = 160.0  
#         self.kd = 8.0

#         # Autonomous Navigation Properties
#         self.navigation_state = 'MANUAL' 
#         self.target_x = 0.0
#         self.target_y = 0.0

#         # Debugging Metadata Cache
#         self.last_received_cmd_source = "None"
#         self.raw_cmd_vel = [0.0, 0.0, 0.0]

#         self.current_velocity = [0.0, 0.0, 0.0] 
#         self.target_velocity = [0.0, 0.0, 0.0] 

#         # Subscriptions
#         self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.cmd_sub = self.create_subscription(String, '/caddie/text_command', self.text_command_callback, 10)
#         self.ball_sub = self.create_subscription(PoseStamped, '/caddie/ball_detections', self.ball_detection_callback, 10)
        
#         self.get_logger().info("Successfully initialized all unified control subscribers via Zenoh!")

#     def cmd_vel_callback(self, msg):
#         self.raw_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
#         self.last_received_cmd_source = "/cmd_vel (Teleop)"

#         if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
#             if self.navigation_state != 'MANUAL':
#                 self.get_logger().warn("Manual override detected! Exiting autonomous routing mode.")
#                 self.navigation_state = 'MANUAL'
        
#         if self.navigation_state == 'MANUAL':
#             self.target_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

#     def text_command_callback(self, msg):
#         command = msg.data.lower()
#         self.last_received_cmd_source = f"/caddie/text_command ('{msg.data}')"

#         if 'return home' in command or 'home' in command:
#             self.get_logger().info("Action Triggered: Calculating Autonomous Return Vector to Origin (0,0)...")
#             self.target_x = 0.0
#             self.target_y = 0.0
#             self.navigation_state = 'RETURNING_HOME'
            
#         elif 'retrieve' in command:
#             self.get_logger().info("Action Triggered: Scanning environment coordinates for ball retrieval.")
#             try:
#                 ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "golf_ball")
#                 self.target_x = self.data.xpos[ball_body_id][0]
#                 self.target_y = self.data.xpos[ball_body_id][1]
#                 self.navigation_state = 'TRACKING_BALL'
#                 self.get_logger().info(f"Target Ball locked at coordinates: [{self.target_x:.2f}, {self.target_y:.2f}]")
#             except Exception as e:
#                 self.get_logger().error(f"Perception error querying model parameters: {e}")

#     def ball_detection_callback(self, msg):
#         self.last_received_cmd_source = "/caddie/ball_detections"
#         if self.navigation_state == 'MANUAL':
#             self.target_x = msg.pose.position.x
#             self.target_y = msg.pose.position.y
#             self.navigation_state = 'TRACKING_BALL'

#     def run_simulation(self):
#         self.get_logger().info("Starting MuJoCo Passive Viewer...")
        
#         # Base setup parameters
#         self.data.qpos[0] = 0.0
#         self.data.qpos[1] = 0.0
#         self.data.qpos[2] = 0.44  
#         self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        
#         gait_frequency = 2.5  
#         step_height = 0.18

#         with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
#             self.get_logger().info("Go2 is spawned in Dynamic Golf Environment! Waiting for instructions...")
            
#             start_time = time.time()
            
#             while viewer.is_running():
#                 step_start = time.time()
#                 rclpy.spin_once(self, timeout_sec=0.001)

#                 # 1. Live Trajectory & Autonomy Tracking System
#                 robot_x = self.data.qpos[0]
#                 robot_y = self.data.qpos[1]
                
#                 # Extract robot heading (yaw) from internal quaternion data
#                 q0, q1, q2, q3 = self.data.qpos[3:7]
#                 robot_yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

#                 local_angle = 0.0
#                 distance = 0.0

#                 if self.navigation_state in ['RETURNING_HOME', 'TRACKING_BALL']:
#                     dx = self.target_x - robot_x
#                     dy = self.target_y - robot_y
#                     distance = np.sqrt(dx**2 + dy**2)
                    
#                     if distance < 0.22:
#                         self.get_logger().info(f"Goal Complete! Destination reached. Braking base.")
#                         self.target_velocity = [0.0, 0.0, 0.0]
#                         self.navigation_state = 'MANUAL'
#                     else:
#                         global_heading = np.arctan2(dy, dx)
#                         local_angle = global_heading - robot_yaw
#                         local_angle = np.arctan2(np.sin(local_angle), np.cos(local_angle))

#                         if abs(local_angle) > 0.78:
#                             vx = 0.0
#                             vy = 0.0
#                             wz = min(0.4, 0.6 * local_angle)
#                         else:
#                             vx = min(0.25, 0.4 * distance * np.cos(local_angle))
#                             vy = min(0.15, 0.3 * distance * np.sin(local_angle))
#                             wz = min(0.3, 0.5 * local_angle)
                        
#                         self.target_velocity = [vx, vy, wz]

#                 # --- 📈 ACCELERATION RAMP FILTER FOR STABILITY 📈 ---
#                 ramp_rate_linear = 0.005  
#                 ramp_rate_angular = 0.01  
                
#                 self.current_velocity[0] += np.clip(self.target_velocity[0] - self.current_velocity[0], -ramp_rate_linear, ramp_rate_linear)
#                 self.current_velocity[1] += np.clip(self.target_velocity[1] - self.current_velocity[1], -ramp_rate_linear, ramp_rate_linear)
#                 self.current_velocity[2] += np.clip(self.target_velocity[2] - self.current_velocity[2], -ramp_rate_angular, ramp_rate_angular)

#                 # --- 🎛️ INTEGRATED TERMINAL DEBUGGER (THROTTLED AT 1HZ) 🎛️ ---
#                 self.get_logger().info(
#                     f"\n[DEBUGGER] State: {self.navigation_state} | Pos: [{robot_x:.2f}, {robot_y:.2f}] | Yaw: {robot_yaw:.2f}rad\n"
#                     f"           Target: [{self.target_x:.2f}, {self.target_y:.2f}] (Dist: {distance:.2f}m, Err: {local_angle:.2f}rad)\n"
#                     f"           Source: {self.last_received_cmd_source} | Raw Twist: X:{self.raw_cmd_vel[0]:.2f} Y:{self.raw_cmd_vel[1]:.2f} Wz:{self.raw_cmd_vel[2]:.2f}\n"
#                     f"           Output: X:{self.current_velocity[0]:.3f} Y:{self.current_velocity[1]:.3f} Yaw:{self.current_velocity[2]:.3f}",
#                     throttle_duration_sec=1.0
#                 )

#                 # 2. Quadruped Joint Kinematics Processing Loop
#                 t = time.time() - start_time
#                 vx, vy, wz = self.current_velocity

#                 phase_1 = 2 * np.pi * gait_frequency * t
#                 phase_2 = phase_1 + np.pi

#                 for actuator_name, target_pos in self.stand_targets.items():
#                     try:
#                         actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                        
#                         if actuator_id != -1:
#                             current_pos = self.data.actuator_length[actuator_id]
#                             current_vel = self.data.actuator_velocity[actuator_id]
                            
#                             dynamic_target = target_pos
                            
#                             if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01):
#                                 is_phase_1 = actuator_name in ['FL_hip', 'FL_thigh', 'FL_calf', 'RR_hip', 'RR_thigh', 'RR_calf']
#                                 current_phase = phase_1 if is_phase_1 else phase_2
                                
#                                 is_left_side = 'FL' in actuator_name or 'RL' in actuator_name
#                                 is_right_side = 'FR' in actuator_name or 'RR' in actuator_name
                                
#                                 # Directional Multiplier for symmetric center rotation 
#                                 side_sign = 1.0 if is_left_side else -1.0

#                                 # Hip Joints
#                                 if '_hip' in actuator_name:
#                                     dynamic_target += np.sin(current_phase) * step_height * vy
#                                     dynamic_target += np.sin(current_phase) * step_height * wz * side_sign

#                                 # Thigh Joints
#                                 elif '_thigh' in actuator_name:
#                                     dynamic_target += np.sin(current_phase) * step_height * vx
#                                     dynamic_target += np.sin(current_phase) * step_height * wz * side_sign

#                                 # Calf Joints
#                                 elif '_calf' in actuator_name:
#                                     dynamic_target += np.cos(current_phase) * step_height * abs(vx) * 1.5
#                                     dynamic_target += np.cos(current_phase) * step_height * abs(vy) * 1.5
#                                     dynamic_target += np.cos(current_phase) * step_height * abs(wz) * 1.5

#                             torque = self.kp * (dynamic_target - current_pos) - self.kd * current_vel
#                             self.data.ctrl[actuator_id] = torque
#                     except Exception:
#                         continue

#                 mujoco.mj_step(self.model, self.data)
#                 viewer.sync()

#                 time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
#                 if time_until_next_step > 0:
#                     time.sleep(time_until_next_step)

# def main(args=None):
#     rclpy.init(args=args)
#     caddie_sim = Go2MujocoWalkBridge()
#     try:
#         caddie_sim.run_simulation()
#     except KeyboardInterrupt:
#         print("\nShutting down simulation...")
#     finally:
#         caddie_sim.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import math
import os

class Go2MujocoWalkBridge(Node):
    def __init__(self):
        super().__init__('caddie_mujoco_bridge')
        
        print("Creating Dynamic Golf Environment in MuJoCo...")
        
        # --- ⛳ LOAD DYNAMIC GOLF SCENE FROM XML ⛳ ---
        xml_path = os.path.join(os.path.dirname(__file__), 'dynamic_golf_scene.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Stance configurations
        self.stand_targets = {
            'FR_hip': 0.0, 'FR_thigh': 0.6, 'FR_calf': -1.2,
            'FL_hip': 0.0, 'FL_thigh': 0.6, 'FL_calf': -1.2,
            'RR_hip': 0.0, 'RR_thigh': 0.6, 'RR_calf': -1.2,
            'RL_hip': 0.0, 'RL_thigh': 0.6, 'RL_calf': -1.2
        }
        
        self.kp = 160.0  
        self.kd = 8.0

        # Autonomous Navigation & Localized Search Properties
        self.navigation_state = 'MANUAL' 
        self.target_x = 0.0
        self.target_y = 0.0
        self.search_start_time = 0.0

        # Simulated Tracking Parameters (Onboard Camera Simulation)
        self.max_vision_distance = 4.0     
        self.vision_fov_rad = math.radians(60.0) 
        self.ball_is_spotted = False

        # Debugging Metadata Cache
        self.last_received_cmd_source = "None"
        self.raw_cmd_vel = [0.0, 0.0, 0.0]

        self.current_velocity = [0.0, 0.0, 0.0] 
        self.target_velocity = [0.0, 0.0, 0.0] 

        # Subscriptions
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/caddie/text_command', self.text_command_callback, 10)
        self.ball_sub = self.create_subscription(PoseStamped, '/caddie/ball_detections', self.ball_detection_callback, 10)
        
        self.get_logger().info("Successfully initialized Subscribers with Separated Scan-on-Command Logic!")

    def cmd_vel_callback(self, msg):
        self.raw_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_received_cmd_source = "/cmd_vel (Teleop)"

        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            if self.navigation_state != 'MANUAL':
                self.get_logger().warn("Manual override detected! Exiting autonomous loops.")
                self.navigation_state = 'MANUAL'
        
        if self.navigation_state == 'MANUAL':
            self.target_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def text_command_callback(self, msg):
        command = msg.data.lower()
        self.last_received_cmd_source = f"/caddie/text_command ('{msg.data}')"

        if 'return home' in command or 'home' in command:
            self.get_logger().info("Action Triggered: Returning to home origin (0,0)...")
            self.target_x = 0.0
            self.target_y = 0.0
            self.navigation_state = 'RETURNING_HOME'
            
        elif 'retrieve' in command:
            self.get_logger().info("Action Triggered: Commencing localized radar scan sweep for ball detection...")
            self.navigation_state = 'LOCAL_SEARCH'
            self.search_start_time = time.time()

    def ball_detection_callback(self, msg):
        self.last_received_cmd_source = "/caddie/ball_detections"
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.navigation_state = 'WAYPOINT_APPROACH'
        self.get_logger().info(f"Rough target coordinates queued: [{self.target_x:.2f}, {self.target_y:.2f}]. Approaching area...")

    def run_simulation(self):
        self.get_logger().info("Starting MuJoCo Passive Viewer...")
        
        # Base setup parameters
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.44  
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        
        gait_frequency = 2.5  
        step_height = 0.18

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()
            
            while viewer.is_running():
                step_start = time.time()
                rclpy.spin_once(self, timeout_sec=0.001)

                # State Telemetry Extraction
                robot_x = self.data.qpos[0]
                robot_y = self.data.qpos[1]
                
                q0, q1, q2, q3 = self.data.qpos[3:7]
                robot_yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

                # --- 📷 SIMULATED ON-BOARD CAMERA FOV PIPELINE 📷 ---
                try:
                    ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "golf_ball")
                    real_ball_x = self.data.xpos[ball_body_id][0]
                    real_ball_y = self.data.xpos[ball_body_id][1]
                    dist_to_ball = np.sqrt((real_ball_x - robot_x)**2 + (real_ball_y - robot_y)**2)
                    
                    global_ball_heading = np.arctan2(real_ball_y - robot_y, real_ball_x - robot_x)
                    camera_angle_err = global_ball_heading - robot_yaw
                    camera_angle_err = np.arctan2(np.sin(camera_angle_err), np.cos(camera_angle_err))
                    
                    if dist_to_ball <= self.max_vision_distance and abs(camera_angle_err) <= self.vision_fov_rad:
                        self.ball_is_spotted = True
                        
                        # Trigger target override strictly when local searching is enabled
                        if self.navigation_state == 'LOCAL_SEARCH':
                            self.target_x = real_ball_x
                            self.target_y = real_ball_y
                            self.navigation_state = 'FINAL_INTERCEPT'
                            self.get_logger().info(f"🎯 TRUE LOCK ACQUIRED! Tracking exact ball frame: [{self.target_x:.2f}, {self.target_y:.2f}]")
                    else:
                        self.ball_is_spotted = False
                except Exception:
                    dist_to_ball = 0.0
                    camera_angle_err = 0.0
                    self.ball_is_spotted = False

                # 2. Universal Navigation Calculation Node Block
                dx = self.target_x - robot_x
                dy = self.target_y - robot_y
                distance = np.sqrt(dx**2 + dy**2)
                
                global_heading = np.arctan2(dy, dx)
                local_angle = global_heading - robot_yaw
                local_angle = np.arctan2(np.sin(local_angle), np.cos(local_angle)) # Globally normalized to ±PI

                # 3. Autonomy State Machine Vector Execution
                if self.navigation_state == 'WAYPOINT_APPROACH':
                    if distance < 0.30: 
                        self.get_logger().info("📍 Arrived at rough target area. Halting and entering STANDBY. Awaiting 'retrieve' command to scan...")
                        self.target_velocity = [0.0, 0.0, 0.0]
                        self.navigation_state = 'STANDBY_AT_WAYPOINT'
                    else:
                        if abs(local_angle) > 0.45: # Pivot first
                            self.target_velocity = [0.0, 0.0, np.clip(0.6 * local_angle, -0.45, 0.45)]
                        else:
                            self.target_velocity = [
                                min(0.45, 0.6 * distance) * np.cos(local_angle),
                                min(0.15, 0.3 * distance * np.sin(local_angle)),
                                np.clip(0.5 * local_angle, -0.30, 0.30)
                            ]

                elif self.navigation_state == 'STANDBY_AT_WAYPOINT':
                    self.target_velocity = [0.0, 0.0, 0.0]

                elif self.navigation_state == 'LOCAL_SEARCH':
                    elapsed_search_time = time.time() - self.search_start_time
                    growth_factor = 0.05  
                    spiral_radius = min(2.0, growth_factor * elapsed_search_time)
                    
                    self.target_velocity = [
                        0.12,                            
                        min(0.15, spiral_radius * 0.12), 
                        0.35                             
                    ]

                # PHASE 4: Head-first true camera tracking approach vector
                elif self.navigation_state == 'FINAL_INTERCEPT':
                    dx = self.target_x - robot_x
                    dy = self.target_y - robot_y
                    distance = np.sqrt(dx**2 + dy**2)
                    
                    if distance < 0.20:
                        self.get_logger().info(f"⚽ BALL SECURED! Operation complete.")
                        self.target_velocity = [0.0, 0.0, 0.0]
                        self.navigation_state = 'MANUAL'
                    else:
                        global_heading = np.arctan2(dy, dx)
                        local_angle = global_heading - robot_yaw
                        local_angle = np.arctan2(np.sin(local_angle), np.cos(local_angle)) # Normalize to ±PI
                        
                        # --- 🎯 THE ALIGNMENT LOCK FIX 🎯 ---
                        # If the ball is more than 20 degrees to the side, freeze forward walk 
                        # and rotate gracefully to face it head-on so it never flies out of camera FOV
                        if abs(local_angle) > 0.35:
                            vx = 0.0
                            vy = 0.0
                            # Softened turning gain (0.4) to smoothly approach the center line without overshooting
                            wz = np.clip(0.4 * local_angle, -0.25, 0.25)
                        else:
                            # Ball is cleanly centered in the camera! Run straight to it
                            vx = min(0.40, 0.6 * distance) * max(0.1, np.cos(local_angle))
                            vy = min(0.12, 0.25 * distance * np.sin(local_angle))
                            wz = np.clip(0.3 * local_angle, -0.15, 0.15)
                        
                        self.target_velocity = [vx, vy, wz]
                elif self.navigation_state == 'RETURNING_HOME':
                    # Home coordinate calculations
                    dx_home = 0.0 - robot_x
                    dy_home = 0.0 - robot_y
                    dist_home = np.sqrt(dx_home**2 + dy_home**2)
                    local_angle_home = np.arctan2(np.sin(np.arctan2(dy_home, dx_home) - robot_yaw), np.cos(np.arctan2(dy_home, dx_home) - robot_yaw))
                    
                    if dist_home < 0.22:
                        self.get_logger().info(f"Arrived safely back at home base.")
                        self.target_velocity = [0.0, 0.0, 0.0]
                        self.navigation_state = 'MANUAL'
                    else:
                        self.target_velocity = [
                            min(0.45, 0.6 * dist_home) * max(0.0, np.cos(local_angle_home)),
                            min(0.15, 0.3 * dist_home * np.sin(local_angle_home)),
                            np.clip(0.5 * local_angle_home, -0.35, 0.35)
                        ]

                # --- 📈 ACCELERATION RAMP FILTER FOR STABILITY ---
                ramp_rate_linear = 0.020   
                ramp_rate_angular = 0.040  
                self.current_velocity[0] += np.clip(self.target_velocity[0] - self.current_velocity[0], -ramp_rate_linear, ramp_rate_linear)
                self.current_velocity[1] += np.clip(self.target_velocity[1] - self.current_velocity[1], -ramp_rate_linear, ramp_rate_linear)
                self.current_velocity[2] += np.clip(self.target_velocity[2] - self.current_velocity[2], -ramp_rate_angular, ramp_rate_angular)

                # --- 🎛️ SYSTEM TELEMETRY DISPLAY LOGGER ---
                vis_status = "👁️ LOCKED ON BALL" if self.ball_is_spotted else "🔍 EXPLORING AREA"
                self.get_logger().info(
                    f"\n[CADDIE ACTIVE HUD] State Flow : {self.navigation_state} | Vision Scope: {vis_status}\n"
                    f"                    Robot Position Framework: [{robot_x:.2f}, {robot_y:.2f}] | Waypoint Objective: [{self.target_x:.2f}, {self.target_y:.2f}]\n"
                    f"                    Velocity Matrix Ramped   -> Linear X: {self.current_velocity[0]:.3f} | Angular Yaw: {self.current_velocity[2]:.3f}",
                    throttle_duration_sec=1.0
                )

                # 4. Quadruped Joint Kinematics Processing Loop
                t = time.time() - start_time
                vx, vy, wz = self.current_velocity
                phase_1 = 2 * np.pi * gait_frequency * t
                phase_2 = phase_1 + np.pi

                for actuator_name, target_pos in self.stand_targets.items():
                    try:
                        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                        if actuator_id != -1:
                            current_pos = self.data.actuator_length[actuator_id]
                            current_vel = self.data.actuator_velocity[actuator_id]
                            dynamic_target = target_pos
                            
                            if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01):
                                is_phase_1 = actuator_name in ['FL_hip', 'FL_thigh', 'FL_calf', 'RR_hip', 'RR_thigh', 'RR_calf']
                                current_phase = phase_1 if is_phase_1 else phase_2
                                is_left_side = 'FL' in actuator_name or 'RL' in actuator_name
                                side_sign = 1.0 if is_left_side else -1.0

                                if '_hip' in actuator_name:
                                    dynamic_target += np.sin(current_phase) * step_height * vy
                                    dynamic_target += np.sin(current_phase) * step_height * wz * side_sign
                                elif '_thigh' in actuator_name:
                                    dynamic_target += np.sin(current_phase) * step_height * vx
                                    dynamic_target += np.sin(current_phase) * step_height * wz * side_sign
                                elif '_calf' in actuator_name:
                                    dynamic_target += np.cos(current_phase) * step_height * abs(vx) * 1.5
                                    dynamic_target += np.cos(current_phase) * step_height * abs(vy) * 1.5
                                    dynamic_target += np.cos(current_phase) * step_height * abs(wz) * 1.5

                            self.data.ctrl[actuator_id] = self.kp * (dynamic_target - current_pos) - self.kd * current_vel
                    except Exception:
                        continue

                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

def main(args=None):
    rclpy.init(args=args)
    caddie_sim = Go2MujocoWalkBridge()
    try:
        caddie_sim.run_simulation()
    except KeyboardInterrupt:
        print("\nShutting down simulation...")
    finally:
        caddie_sim.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()