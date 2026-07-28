#!/usr/bin/env python3
import os
import time
import math
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node

from camera_pipeline import SimulatedCameraPipeline
from navigation_sm import NavigationStateMachine
from gait_generator import TrotGaitGenerator
from dog_actions import DogActionsController

class Go2MujocoWalkBridge(Node):
    def __init__(self):
        super().__init__('caddie_mujoco_bridge')

        xml_path = os.path.join(os.path.dirname(__file__), 'dynamic_golf_scene.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.camera = SimulatedCameraPipeline()
        self.nav_sm = NavigationStateMachine(self.get_logger())
        self.gait = TrotGaitGenerator(step_height=0.2, gait_frequency=2.0)

        self.current_velocity = [0.0, 0.0, 0.0]
        self.target_velocity = [0.0, 0.0, 0.0]

        # Initialize Actions Controller
        self.actions = DogActionsController(self, self.model, self.data, self.nav_sm)

        self.get_logger().info("Modularized Bridge with Auto-Fetch Initialized! Hit the ball to start.")

    def run_simulation(self):
        self.data.qpos[0:3] = [0.0, 0.0, 0.44]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        trunk_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()

            while viewer.is_running():
                step_start = time.time()
                rclpy.spin_once(self, timeout_sec=0.001)

                rx, ry, rz = self.data.qpos[0], self.data.qpos[1], self.data.qpos[2]
                q0, q1, q2, q3 = self.data.qpos[3:7]
                ryaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

                # Camera Tracking
                spotted, bx, by, dist, err = self.camera.track_ball(self.model, self.data, rx, ry, ryaw)
                if spotted:
                    self.nav_sm.target_x = bx
                    self.nav_sm.target_y = by

                # Navigation Velocity Computation
                if self.nav_sm.state != 'MANUAL':
                    self.target_velocity = self.nav_sm.compute_target_velocity(rx, ry, ryaw, spotted, bx, by)

                # 🎯 Auto-Fetch & Attachment Logic 🎯
                self.actions.monitor_ball_and_autofetch() # 👈 අලුතෙන් දැම්ම Function එක!
                self.actions.update_ball_attachment()

                # Velocity Ramp Filters
                self.current_velocity[0] += np.clip(self.target_velocity[0] - self.current_velocity[0], -0.02, 0.02)
                self.current_velocity[1] += np.clip(self.target_velocity[1] - self.current_velocity[1], -0.02, 0.02)
                self.current_velocity[2] += np.clip(self.target_velocity[2] - self.current_velocity[2], -0.04, 0.04)

                # Torque Kinematics Engine
                self.gait.compute_and_apply_torques(self.model, self.data, self.current_velocity, time.time() - start_time)

                mujoco.mj_step(self.model, self.data)

                # Visual overlay rendering
                if viewer.user_scn:
                    viewer.user_scn.ngeom = 0
                    dog_pos = self.data.xpos[trunk_id] if trunk_id != -1 else np.array([rx, ry, rz])
                    R = self.data.xmat[trunk_id].reshape(3, 3) if trunk_id != -1 else np.eye(3)

                    axis_len = 3.0
                    axis_thick = 0.005
                    fwd_vec, left_vec, up_vec = R[:, 0], R[:, 1], R[:, 2]

                    # Render local RGB axes
                    mujoco.mjv_connector(viewer.user_scn.geoms[viewer.user_scn.ngeom], type=mujoco.mjtGeom.mjGEOM_CYLINDER, width=axis_thick, from_=dog_pos, to=dog_pos + fwd_vec * axis_len)
                    viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array([1.0, 0.0, 0.0, 0.8])
                    viewer.user_scn.ngeom += 1

                    mujoco.mjv_connector(viewer.user_scn.geoms[viewer.user_scn.ngeom], type=mujoco.mjtGeom.mjGEOM_CYLINDER, width=axis_thick, from_=dog_pos, to=dog_pos + left_vec * axis_len)
                    viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array([0.0, 1.0, 0.0, 0.8])
                    viewer.user_scn.ngeom += 1

                    mujoco.mjv_connector(viewer.user_scn.geoms[viewer.user_scn.ngeom], type=mujoco.mjtGeom.mjGEOM_CYLINDER, width=axis_thick, from_=dog_pos, to=dog_pos + up_vec * 0.4)
                    viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array([0.0, 0.5, 1.0, 0.8])
                    viewer.user_scn.ngeom += 1

                    # Metric Ticks & Labels
                    for step in range(1, 31):
                        dist_m = step * 0.1
                        fwd_pos = dog_pos + fwd_vec * dist_m
                        fwd_pos[2] = 0.005

                        mujoco.mjv_connector(viewer.user_scn.geoms[viewer.user_scn.ngeom], type=mujoco.mjtGeom.mjGEOM_CYLINDER, width=0.003, from_=fwd_pos - left_vec * 0.04, to=fwd_pos + left_vec * 0.04)
                        viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array([1.0, 0.3, 0.3, 0.7])
                        viewer.user_scn.ngeom += 1

                        if step % 5 == 0:
                            geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                            mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_LABEL, size=np.array([0.0, 0.0, 0.0]), pos=fwd_pos + np.array([0.0, 0.0, 0.03]), mat=np.eye(3).flatten(), rgba=np.array([1.0, 0.2, 0.2, 1.0]))
                            geom.label = f"+X {dist_m:.1f}m"
                            viewer.user_scn.ngeom += 1

                    # Ball tracking line
                    if spotted or self.nav_sm.ball_grabbed:
                        mujoco.mjv_connector(viewer.user_scn.geoms[viewer.user_scn.ngeom], type=mujoco.mjtGeom.mjGEOM_CYLINDER, width=0.005, from_=dog_pos, to=np.array([bx, by, 0.05]))
                        viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array([1.0, 1.0, 0.0, 0.8])
                        viewer.user_scn.ngeom += 1

                viewer.sync()

                sleep_time = self.model.opt.timestep - (time.time() - step_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    bridge = Go2MujocoWalkBridge()
    try:
        bridge.run_simulation()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()