#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        # Create a publisher to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1) # Give ROS 2 a second to register the publisher
        
    def send_cmd(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=2.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        
        self.get_logger().info(
            f"Publishing: Linear[x: {linear_x}, y: {linear_y}], Angular[z: {angular_z}] for {duration}s"
        )
        
        # Publish continuously during the duration interval to keep the robot moving
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            self.publisher_.publish(msg)
            time.sleep(0.1) # 10 Hz publishing rate
            
    def stop_robot(self):
        self.get_logger().info("Stopping robot...")
        msg = Twist() # All fields default to 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mover = RobotMover()
    
    try:
        # 1. Go forward (X direction)
        mover.send_cmd(linear_x=0.5, duration=2.0)
        
        # 2. Go backward (-X direction)
        mover.send_cmd(linear_x=-0.5, duration=2.0)
        
        # 3. Move +Y direction (Left)
        mover.send_cmd(linear_y=0.5, duration=2.0)
        
        # 4. Move -Y direction (Right)
        mover.send_cmd(linear_y=-0.5, duration=2.0)
        
        # 5. Rotate +Yaw (Counter-Clockwise)
        mover.send_cmd(angular_z=0.5, duration=10.0)
        
        # 6. Rotate -Yaw (Clockwise)
        mover.send_cmd(angular_z=-0.5, duration=-10.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        # Always stop the robot before exiting!
        mover.stop_robot()
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()