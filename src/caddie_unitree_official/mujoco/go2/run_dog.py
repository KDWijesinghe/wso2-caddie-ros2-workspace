# #!/usr/bin/env python3
# import mujoco
# import mujoco.viewer
# import time
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class Go2MujocoWalkBridge(Node):
#     def __init__(self):
#         super().__init__('caddie_mujoco_bridge')
        
#         print("Loading MuJoCo Scene...")
#         self.model = mujoco.MjModel.from_xml_path('golf_course.xml')
#         self.data = mujoco.MjData(self.model)
        
#         # මූලිකව හිටගෙන ඉන්න Angles
#         self.stand_targets = {
#             'FR_hip': 0.0, 'FR_thigh': 0.6, 'FR_calf': -1.2,
#             'FL_hip': 0.0, 'FL_thigh': 0.6, 'FL_calf': -1.2,
#             'RR_hip': 0.0, 'RR_thigh': 0.6, 'RR_calf': -1.2,
#             'RL_hip': 0.0, 'RL_thigh': 0.6, 'RL_calf': -1.2
#         }
        
#         self.kp = 160.0  
#         self.kd = 8.0

#         # [X වේගය, Y වේගය, Yaw වේගය]
#         self.current_velocity = [0.0, 0.0, 0.0] 

#         self.subscriber = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         print("Successfully subscribed to ROS2 /cmd_vel via Zenoh!")

#     def cmd_vel_callback(self, msg):
#         print(f"Received Command -> X: {msg.linear.x:.2f}, Y: {msg.linear.y:.2f}, Yaw: {msg.angular.z:.2f}")
#         self.current_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

#     def run_simulation(self):
#         print("Starting MuJoCo Passive Viewer...")
        
# # --- බල්ලව කෙලින්ම අර පිටිපස්සේ තියෙන Terrain එක උඩට Spawn කරමු ---
#         self.data.qpos[0] = -1.5  # X එක -1.5 හෝ -2.0 දක්වා පස්සට ගන්න (Terrain එක තියෙන තැන)
#         self.data.qpos[1] = 1.5   # Y එක 1.5ක් වගේ උඩට ගන්න
#         self.data.qpos[2] = 0.75  # Z එක (Height) 0.75m තියන්න කන්ද උඩට වැටෙන්න
#         self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        
#         gait_frequency = 2.5  
#         step_height = 0.25     

#         with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
#             print("\nGo2 is ready for BACKWARD movement! Send /cmd_vel to test.")
            
#             start_time = time.time()
            
#             while viewer.is_running():
#                 step_start = time.time()
#                 rclpy.spin_once(self, timeout_sec=0.001)

#                 t = time.time() - start_time
#                 vx = self.current_velocity[0] # Linear X
#                 vy = self.current_velocity[1] # Linear Y
#                 wz = self.current_velocity[2] # Angular Z (Yaw)

#                 # Diagonal Phases
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
                                
#                                 # --- 1. HIP JOINTS ಪාලනය ---
#                                 if '_hip' in actuator_name:
#                                     dynamic_target += np.sin(current_phase) * step_height * vy
#                                     if wz != 0.0 and is_left_side:
#                                         dynamic_target += np.sin(current_phase) * step_height * wz

#                                 # --- 2. THIGH JOINTS ಪාලනය ---
#                                 elif '_thigh' in actuator_name:
#                                     # vx සෘණ වුණාම Thigh එක ඔටෝම පස්සට swing වෙන්න පටන් ගනියි (මේක හරි)
#                                     dynamic_target += np.sin(current_phase) * step_height * vx
#                                     if wz != 0.0 and is_right_side:
#                                         dynamic_target += np.sin(current_phase) * step_height * wz

#                                 # --- 3. CALF JOINTS පාලනය (FIXED FOR BACKWARD) ---
#                                 elif '_calf' in actuator_name:
#                                     # FIXED: vx වෙනුවට abs(vx) දැමීමෙන් පස්සට යද්දීත් කකුල උඩට එසවීම නිවැරදිව සිදුවේ!
#                                     dynamic_target += np.cos(current_phase) * step_height * abs(vx) * 1.5
#                                     dynamic_target += np.cos(current_phase) * step_height * abs(vy) * 1.5
#                                     if wz != 0.0 and is_right_side:
#                                         dynamic_target += np.cos(current_phase) * step_height * wz * 1.5

#                             # PD Control Loop
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
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Go2MujocoWalkBridge(Node):
    def __init__(self):
        super().__init__('caddie_mujoco_bridge')
        
        print("Creating Dynamic Golf Environment in MuJoCo...")
        
        # --- ⛳ PYTHON DYNAMIC GOLF SCENE STRING (FIXED RGBA) ⛳ ---
        golf_scene_xml = """
        <mujoco model="dynamic_golf_scene">
            <include file="go2.xml"/>

            <statistic center="0 0 0.1" extent="0.8"/>

            <visual>
                <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
                <rgba haze="0.15 0.25 0.35 1"/>
                <global azimuth="-130" elevation="-20"/>
            </visual>

            <asset>
                <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
                <texture type="2d" name="grass_plane" builtin="checker" mark="edge" rgb1="0.13 0.55 0.13" rgb2="0.1 0.45 0.1"
                  markrgb="0.2 0.6 0.2" width="300" height="300"/>
                <material name="grass_plane" texture="grass_plane" texuniform="true" texrepeat="5 5" reflectance="0.1"/>
            </asset>

            <worldbody>
                <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
                
                <geom name="floor" size="0 0 0.05" type="plane" material="grass_plane" friction="0.6 0.005 0.0001"/>

                <geom name="sand_bunker" type="box" size="2 2 0.005" pos="3 2 0.002" rgba="0.85 0.75 0.55 1" friction="1.2 0.01 0.001"/>

                <body name="golf_hole" pos="4 4 0">
                    <geom type="cylinder" size="0.12 0.001" rgba="0.1 0.1 0.1 1" pos="0 0 0.001"/>
                    <geom type="cylinder" size="0.015 1.0" pos="0 0 0.5" rgba="0.8 0.8 0.8 1"/>
                    <geom type="box" size="0.18 0.005 0.12" pos="0.15 0 0.9" rgba="0.9 0.1 0.1 1"/>
                </body>

                <body name="golf_ball" pos="1.5 0.0 0.05">
                    <freejoint name="ball_joint"/>
                    <geom name="ball_geom" type="sphere" size="0.04" mass="0.045" rgba="1 1 1 1" friction="0.4 0.005 0.0001"/>
                </body>
            </worldbody>
        </mujoco>
        """
        
        # XML String එකෙන් model එක load කිරීම (Bypassing scene.xml)
        self.model = mujoco.MjModel.from_xml_string(golf_scene_xml)
        self.data = mujoco.MjData(self.model)
        
        # මූලිකව හිටගෙන ඉන්න Angles
        self.stand_targets = {
            'FR_hip': 0.0, 'FR_thigh': 0.6, 'FR_calf': -1.2,
            'FL_hip': 0.0, 'FL_thigh': 0.6, 'FL_calf': -1.2,
            'RR_hip': 0.0, 'RR_thigh': 0.6, 'RR_calf': -1.2,
            'RL_hip': 0.0, 'RL_thigh': 0.6, 'RL_calf': -1.2
        }
        
        self.kp = 160.0  
        self.kd = 8.0

        # [X වේගය, Y වේගය, Yaw වේගය]
        self.current_velocity = [0.0, 0.0, 0.0] 

        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        print("Successfully subscribed to ROS2 /cmd_vel via Zenoh!")

    def cmd_vel_callback(self, msg):
        print(f"Received Command -> X: {msg.linear.x:.2f}, Y: {msg.linear.y:.2f}, Yaw: {msg.angular.z:.2f}")
        self.current_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def run_simulation(self):
        print("Starting MuJoCo Passive Viewer...")
        
        # Initial positions
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.44  
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        
        gait_frequency = 2.5  
        step_height = 0.25     

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            print("\nGo2 is spawned in Dynamic Golf Environment! Send /cmd_vel to test.")
            
            start_time = time.time()
            
            while viewer.is_running():
                step_start = time.time()
                rclpy.spin_once(self, timeout_sec=0.001)

                t = time.time() - start_time
                vx = self.current_velocity[0] 
                vy = self.current_velocity[1] 
                wz = self.current_velocity[2] 

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
                                is_right_side = 'FR' in actuator_name or 'RR' in actuator_name
                                
                                # Hip Joints
                                if '_hip' in actuator_name:
                                    dynamic_target += np.sin(current_phase) * step_height * vy
                                    if wz != 0.0 and is_left_side:
                                        dynamic_target += np.sin(current_phase) * step_height * wz

                                # Thigh Joints
                                elif '_thigh' in actuator_name:
                                    dynamic_target += np.sin(current_phase) * step_height * vx
                                    if wz != 0.0 and is_right_side:
                                        dynamic_target += np.sin(current_phase) * step_height * wz

                                # Calf Joints
                                elif '_calf' in actuator_name:
                                    dynamic_target += np.cos(current_phase) * step_height * abs(vx) * 1.5
                                    dynamic_target += np.cos(current_phase) * step_height * abs(vy) * 1.5
                                    if wz != 0.0 and is_right_side:
                                        dynamic_target += np.cos(current_phase) * step_height * wz * 1.5

                            torque = self.kp * (dynamic_target - current_pos) - self.kd * current_vel
                            self.data.ctrl[actuator_id] = torque
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
        rclpy.shutdown()

if __name__ == "__main__":
    main()