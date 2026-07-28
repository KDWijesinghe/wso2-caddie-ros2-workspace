#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import math
import time

class MujocoBallPicker(Node):
    def __init__(self):
        super().__init__('mujoco_ball_picker')
        
        # States: 'IDLE', 'TRACKING_BALL', 'PICKING_UP', 'COMPLETED'
        self.state = 'IDLE'
        self.ball_x = None
        self.ball_y = None
        
        # Robot position (Assuming starting at 0,0 if odom isn't piped to this frame yet)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Pubs & Subs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/caddie/status', 10)
        
        self.create_subscription(String, '/caddie/text_command', self.command_callback, 10)
        self.create_subscription(PoseStamped, '/caddie/ball_detections', self.ball_detection_callback, 10)
        
        # Control loop timer (Runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("MuJoCo Direct Ball Picker Initialized.")

    def command_callback(self, msg):
        command = msg.data.lower()
        if 'retrieve' in command and self.state == 'IDLE':
            self.get_logger().info("Command received: Tracking nearest ball via direct vectors.")
            self.state = 'TRACKING_BALL'

    def ball_detection_callback(self, msg):
        # Coordinates relative to the robot base
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y

    def control_loop(self):
        if self.state != 'TRACKING_BALL':
            return
            
        if self.ball_x == None or self.ball_y == None:
            self.get_logger().warning("Waiting for ball detection coordinates on /caddie/ball_detections...", throttle_duration_sec=2.0)
            return

        # Calculate distance and angle (yaw) error to target
        distance = math.sqrt(self.ball_x**2 + self.ball_y**2)
        angle_to_ball = math.atan2(self.ball_y, self.ball_x)
        
        twist = Twist()
        
        # Check if we arrived at the ball capture threshold (e.g., 0.2 meters)
        if distance < 0.25:
            self.get_logger().info("Ball reached! Executing collection phase...")
            self.state = 'PICKING_UP'
            self.execute_pickup()
            return

        # Simple P-Controller for steering and approach speed
        # Adjust constants (0.5, 0.3) to change aggression speed
        twist.linear.x = min(0.3, 0.4 * distance * math.cos(angle_to_ball)) 
        twist.linear.y = min(0.2, 0.3 * distance * math.sin(angle_to_ball)) # Holonomic crab-walk adjustment
        twist.angular.z = min(0.4, 0.8 * angle_to_ball)                    # Pivot heading adjustment

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Approaching Ball -> Dist: {distance:.2f}m, Heading Error: {angle_to_ball:.2f} rad", throttle_duration_sec=1.0)

    def execute_pickup(self):
        # Smooth creep forward to engulf the ball
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.5)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Ball secured! Task completed.")
        self.state = 'COMPLETED'

def main(args=None):
    rclpy.init(args=args)
    node = MujocoBallPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()