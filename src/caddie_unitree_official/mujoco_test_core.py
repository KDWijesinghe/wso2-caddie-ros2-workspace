#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import math
import time

class MujocoTestCore(Node):
    def __init__(self):
        super().__init__('mujoco_test_core')
        
        # Internal Odometry (Dead Reckoning)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.last_time = time.time()

        # State Machine: 'IDLE', 'RANDOM_WALK', 'RETURNING_HOME', 'SEEKING_BALL'
        self.state = 'IDLE'
        self.target_x = 0.0
        self.target_y = 0.0

        # Pubs & Subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/caddie/text_command', self.cmd_callback, 10)
        self.create_subscription(PoseStamped, '/caddie/ball_detections', self.ball_callback, 10)
        
        # 10Hz Control and Odometry update loop
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("MuJoCo Test Core Status: Ready. Waiting for commands...")

    def cmd_callback(self, msg):
        cmd = msg.data.lower()
        if 'random' in cmd:
            self.get_logger().info("Starting 5-second random exploratory walk...")
            self.state = 'RANDOM_WALK'
            self.walk_end_time = time.time() + 5.0
        elif 'home' in cmd:
            self.get_logger().info("Command received: Calculating return path to Origin (0,0)...")
            self.target_x = 0.0
            self.target_y = 0.0
            self.state = 'RETURNING_HOME'

    def ball_callback(self, msg):
        # Treat incoming detection as an absolute target position
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.state = 'SEEKING_BALL'
        self.get_logger().info(f"New target locked at absolute coordinates: [{self.target_x}, {self.target_y}]")

    def loop(self):
        # 1. Update internal Odometry approximation
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # We assume the robot is trying to execute the last commanded state 
        # (For true closed-loop, you'd pull this directly out of mujoco sensor data matrix in run_dog.py)
        
        # 2. State Machine Logic
        twist = Twist()
        
        if self.state == 'RANDOM_WALK':
            if time.time() > self.walk_end_time:
                self.get_logger().info("Random walk finished. Standing by.")
                self.state = 'IDLE'
            else:
                # Drift forward and crab side-to-side dynamically
                twist.linear.x = 0.2
                twist.linear.y = 0.1 * math.sin(time.time())
                
                # Update estimated position tracking
                self.pose_x += twist.linear.x * dt
                self.pose_y += twist.linear.y * dt

        elif self.state in ['RETURNING_HOME', 'SEEKING_BALL']:
            # Calculate distance errors
            dx = self.target_x - self.pose_x
            dy = self.target_y - self.pose_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.15:
                self.get_logger().info(f"Target destination reached successfully! State reverted to IDLE.")
                self.state = 'IDLE'
            else:
                # Vector steering calculation
                heading = math.atan2(dy, dx)
                twist.linear.x = min(0.25, 0.4 * distance * math.cos(heading))
                twist.linear.y = min(0.2, 0.4 * distance * math.sin(heading))
                
                # Track position changes
                self.pose_x += twist.linear.x * dt
                self.pose_y += twist.linear.y * dt
                
                self.get_logger().info(f"Pos: [{self.pose_x:.2f}, {self.pose_y:.2f}] -> Target Dist: {distance:.2f}m", throttle_duration_sec=1.5)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoTestCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()