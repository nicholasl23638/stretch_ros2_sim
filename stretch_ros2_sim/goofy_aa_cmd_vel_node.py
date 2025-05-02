# import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy, Image
from std_msgs.msg import Bool, String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from rclpy.impl.logging_severity import LoggingSeverity
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf_transformations import quaternion_from_euler

class CmdVel(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop

        
        self.get_logger().info('Cmd_vel goofy aa initialized')
    
    def control_loop(self):
        self.publish_velocity()
    
    def publish_velocity(self):
        """Publish zero velocity to stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    
    # Create the trajectory follower node
    cmd_vel_node = CmdVel()
    
    # Create the trajectory generator node
    
    # Set up multithreaded executor to run both nodes
    try:
        # Spin the executor
        rclpy.spin(cmd_vel_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cmd_vel_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()