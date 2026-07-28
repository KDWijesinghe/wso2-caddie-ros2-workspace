#!/usr/bin/env python3
import mujoco
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

class DogActionsController:
    def __init__(self, node, model, data, nav_sm):
        self.node = node
        self.model = model
        self.data = data
        self.nav_sm = nav_sm

        # Cache Joint & Body IDs
        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "golf_ball")
        self.trunk_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        self.ball_jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball_joint")
        
        # Auto-fetch State Tracking
        self.ball_was_moving = False

        # ROS 2 Subscriptions
        self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.node.create_subscription(String, '/caddie/text_command', self.text_command_callback, 10)
        self.node.create_subscription(PoseStamped, '/caddie/ball_detections', self.ball_detection_callback, 10)
        self.node.create_subscription(PoseStamped, '/caddie/move_ball', self.move_ball_callback, 10)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            if self.nav_sm.state != 'MANUAL':
                self.node.get_logger().warn("Manual Override triggered.")
                self.nav_sm.state = 'MANUAL'

        if self.nav_sm.state == 'MANUAL':
            self.node.target_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def text_command_callback(self, msg):
        cmd = msg.data.lower()
        if 'get the ball' in cmd or 'retrieve' in cmd or 'fetch' in cmd:
            self.node.get_logger().info("Command Received: FETCH! Executing GOTO_BALL state...")
            self.nav_sm.state = 'GOTO_BALL'
        elif 'track' in cmd or 'watch' in cmd:
            self.node.get_logger().info("Command Received: Watching the ball...")
            self.nav_sm.state = 'TRACKING_VISUAL'
        # 🏌️‍♂️ අලුතෙන් දැම්ම HIT Command එක!
        elif 'hit' in cmd:
            self.node.get_logger().info("🏌️ WHACK! බෝලයට ගැහුවා...")
            if self.ball_jnt_id != -1:
                dof_adr = self.model.jnt_dofadr[self.ball_jnt_id]
                # බෝලයට Physics Velocity එකක් දෙනවා (ඉස්සරහට 3m/s, පැත්තට 1.5m/s)
                self.data.qvel[dof_adr] = 3.0
                self.data.qvel[dof_adr + 1] = 1.5
                self.data.qvel[dof_adr + 2] = 0.0

    def ball_detection_callback(self, msg):
        self.nav_sm.target_x = msg.pose.position.x
        self.nav_sm.target_y = msg.pose.position.y

    def move_ball_callback(self, msg):
        """Teleports the golf ball"""
        if self.ball_jnt_id != -1:
            qpos_idx = self.model.jnt_qposadr[self.ball_jnt_id]
            self.data.qpos[qpos_idx] = msg.pose.position.x
            self.data.qpos[qpos_idx + 1] = msg.pose.position.y
            self.data.qpos[qpos_idx + 2] = 0.05
            
            # ටෙලිපෝට් කරන ගමන් පොඩි Velocity එකක් දෙනවා Auto-Fetch එක අහුවෙන්න!
            dof_adr = self.model.jnt_dofadr[self.ball_jnt_id]
            self.data.qvel[dof_adr] = 0.6  
            self.node.get_logger().info(f"⛳ Ball teleported to: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]")

    def update_ball_attachment(self):
        """Handles physical grabbing & pinning the ball"""
        if self.nav_sm.state == 'GRAB_BALL':
            self.nav_sm.ball_grabbed = True
            self.node.get_logger().info("🎒 බෝලය කටට Lock වුණා! ආපහු Home එකට යනවා...")
            self.nav_sm.state = 'RETURN_WITH_BALL'

        if self.nav_sm.ball_grabbed and self.ball_body_id != -1 and self.trunk_id != -1:
            dog_pos = self.data.xpos[self.trunk_id]
            R = self.data.xmat[self.trunk_id].reshape(3, 3)
            
            mouth_pos = dog_pos + R[:, 0] * 0.20 - R[:, 2] * 0.10
            if self.ball_jnt_id != -1:
                qpos_idx = self.model.jnt_qposadr[self.ball_jnt_id]
                self.data.qpos[qpos_idx:qpos_idx+3] = mouth_pos

    def monitor_ball_and_autofetch(self):
        """Auto-triggers tracking and fetching"""
        if self.ball_jnt_id != -1:
            dof_adr = self.model.jnt_dofadr[self.ball_jnt_id]
            ball_vx = self.data.qvel[dof_adr]
            ball_vy = self.data.qvel[dof_adr + 1]
            speed = np.sqrt(ball_vx**2 + ball_vy**2)

            # 1. බෝලයේ Speed එක 0.5 ට වඩා වැඩි නම් (ගහපු ගමන්)
            if speed > 0.5 and self.nav_sm.state == 'MANUAL':
                self.node.get_logger().info("👀 බෝලය විසි වෙනවා දැක්කා! (Tracking...)")
                self.nav_sm.state = 'TRACKING_VISUAL'
                self.ball_was_moving = True

            # 2. බෝලය නැවතුණාම (Speed 0 වුණාම)
            elif self.nav_sm.state == 'TRACKING_VISUAL' and self.ball_was_moving:
                if speed < 0.05:  # Ball completely stopped
                    self.node.get_logger().info("🛑 බෝලය නැවතුණා! අරන් එන්න පිටත් වෙනවා...")
                    self.nav_sm.state = 'GOTO_BALL'
                    self.ball_was_moving = False