#!/usr/bin/env python3
import numpy as np
import mujoco

class TrotGaitGenerator:
    def __init__(self, kp=160.0, kd=8.0, gait_frequency=2.5, step_height=0.30):
        self.kp = kp
        self.kd = kd
        self.gait_frequency = gait_frequency
        self.step_height = step_height

        self.stand_targets = {
            'FR_hip': 0.0, 'FR_thigh': 0.6, 'FR_calf': -1.2,
            'FL_hip': 0.0, 'FL_thigh': 0.6, 'FL_calf': -1.2,
            'RR_hip': 0.0, 'RR_thigh': 0.6, 'RR_calf': -1.2,
            'RL_hip': 0.0, 'RL_thigh': 0.6, 'RL_calf': -1.2
        }

    def compute_and_apply_torques(self, model, data, current_velocity, elapsed_time):
        vx, vy, wz = current_velocity
        phase_1 = 2 * np.pi * self.gait_frequency * elapsed_time
        phase_2 = phase_1 + np.pi

        for actuator_name, target_pos in self.stand_targets.items():
            try:
                actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                if actuator_id != -1:
                    current_pos = data.actuator_length[actuator_id]
                    current_vel = data.actuator_velocity[actuator_id]
                    dynamic_target = target_pos

                    if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
                        is_phase_1 = actuator_name in ['FL_hip', 'FL_thigh', 'FL_calf', 'RR_hip', 'RR_thigh', 'RR_calf']
                        current_phase = phase_1 if is_phase_1 else phase_2
                        side_sign = 1.0 if ('FL' in actuator_name or 'RL' in actuator_name) else -1.0

                        if '_hip' in actuator_name:
                            dynamic_target += np.sin(current_phase) * self.step_height * vy
                            dynamic_target += np.sin(current_phase) * self.step_height * wz * side_sign
                        elif '_thigh' in actuator_name:
                            dynamic_target += np.sin(current_phase) * self.step_height * vx
                            dynamic_target += np.sin(current_phase) * self.step_height * wz * side_sign
                        elif '_calf' in actuator_name:
                            dynamic_target += np.cos(current_phase) * self.step_height * abs(vx) * 1.5
                            dynamic_target += np.cos(current_phase) * self.step_height * abs(vy) * 1.5
                            dynamic_target += np.cos(current_phase) * self.step_height * abs(wz) * 1.5

                    data.ctrl[actuator_id] = self.kp * (dynamic_target - current_pos) - self.kd * current_vel
            except Exception:
                continue