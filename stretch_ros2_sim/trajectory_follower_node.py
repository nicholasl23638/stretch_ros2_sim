import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        
        # Parameters - can be replaced with ROS2 parameters for runtime configurability
        self.declare_parameter('lookahead_distance', 0.3)  # meters 0.4
        self.declare_parameter('max_linear_speed', 0.3)    # m/s 0.3
        self.declare_parameter('max_angular_speed', 1.9)   # rad/s
        self.declare_parameter('goal_tolerance', 0.1)      # meters
        self.declare_parameter('k_gain', 0.5)              # gain for angular velocity

        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.k_gain = self.get_parameter('k_gain').value
        
        # Subscribers and publishers
        self.trajectory_sub = self.create_subscription(
            Path, 'trajectory', self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.target_point_pub = self.create_publisher(MarkerArray, 'target_markers', 10)
        
        # TF listener setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.trajectory = None
        self.current_pose = None
        self.current_index = 0
        self.is_path_completed = False
        
        # Create timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop
        
        self.get_logger().info('Trajectory follower initialized')
    
    def trajectory_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn('Received trajectory with too few points, ignoring')
            return
            
        self.trajectory = msg
        self.current_index = 0
        self.is_path_completed = False
        self.get_logger().info(f'Received new trajectory with {len(msg.poses)} points')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        if self.trajectory is None or self.current_pose is None:
            return
            
        if self.is_path_completed:
            # Stop the robot when the path is completed
            self.publish_zero_velocity()
            return
            
        # Find the lookahead point on the trajectory
        target_point, target_index = self.find_lookahead_point()
        
        if target_point is None:
            self.get_logger().warn('No suitable target point found')
            self.publish_zero_velocity()
            return
            
        # Calculate control commands using pure pursuit
        cmd = self.calculate_pure_pursuit(target_point)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Visualize the target point
        self.visualize_target(target_point)
        
        # Check if we've reached the end of the trajectory
        if target_index >= len(self.trajectory.poses) - 1:
            # Check if we're close enough to the final point
            final_point = self.trajectory.poses[-1].pose.position
            current_point = self.current_pose.position
            distance = self.calculate_distance(current_point, final_point)
            
            if distance < self.goal_tolerance:
                self.get_logger().info('Trajectory completed!')
                self.is_path_completed = True
    
    def find_lookahead_point(self):
        """Find a point on the trajectory that is approximately lookahead_distance away"""
        if len(self.trajectory.poses) <= self.current_index:
            return None, self.current_index
            
        robot_position = self.current_pose.position
        
        # Start searching from the current index
        min_index = self.current_index
        
        # If we're at the last point, return it
        if min_index >= len(self.trajectory.poses) - 1:
            return self.trajectory.poses[-1].pose.position, len(self.trajectory.poses) - 1
        
        # Find the first point that's at least lookahead_distance away
        target_point = None
        target_index = min_index
        
        for i in range(min_index, len(self.trajectory.poses)):
            point = self.trajectory.poses[i].pose.position
            distance = self.calculate_distance(robot_position, point)
            
            # Update the current index if the point is closer than the lookahead distance
            if distance < self.lookahead_distance:
                self.current_index = i
            
            if distance >= self.lookahead_distance:
                target_point = point
                target_index = i
                break
        
        # If we didn't find a point far enough away, use the last point
        if target_point is None:
            target_point = self.trajectory.poses[-1].pose.position
            target_index = len(self.trajectory.poses) - 1
        
        return target_point, target_index
    
    def calculate_pure_pursuit(self, target_point):
        """Calculate the linear and angular velocities using pure pursuit algorithm"""
        cmd = Twist()
        
        # Get robot's position and orientation
        robot_position = self.current_pose.position
        robot_orientation = self.current_pose.orientation
        
        # Calculate the heading to the target point in the robot's frame
        dx = target_point.x - robot_position.x
        dy = target_point.y - robot_position.y
        
        # Extract yaw from quaternion
        yaw = self.quaternion_to_yaw(robot_orientation)
        
        # Calculate angle to target in robot's frame
        target_angle = math.atan2(dy, dx)
        steering_angle = target_angle - yaw
        
        # Normalize the steering angle
        steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi
        #while steering_angle > math.pi:
        #    steering_angle -= 2 * math.pi
        #while steering_angle < -math.pi:
        #    steering_angle += 2 * math.pi
        
        # Calculate the distance to the target
        distance = self.calculate_distance(robot_position, target_point)
        
        # Apply pure pursuit control law
        # Linear velocity is constant or proportional to distance
        linear_velocity = min(self.max_linear_speed, 0.5 * distance)

        if abs(steering_angle) > math.pi / 2:
            linear_velocity *= -1
            if steering_angle > 0:
                steering_angle -= math.pi
            else:
                steering_angle += math.pi
            # Normalize again in case the angle wraps
            steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi
        
        # Angular velocity is proportional to the steering angle
        angular_velocity = self.k_gain * steering_angle
        
        # Limit the angular velocity
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        
        # Set the velocities
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        
        return cmd
    
    def publish_zero_velocity(self):
        """Publish zero velocity to stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (rotation around z-axis)"""
        # Extract the yaw angle from the quaternion
        # This is a simplified conversion assuming the robot moves in a plane
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def visualize_target(self, target_point):
        """Publish a marker to visualize the target point"""
        marker_array = MarkerArray()
        
        # Target point marker
        target_marker = Marker()
        target_marker.header.frame_id = self.trajectory.header.frame_id
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_point"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target_point.x
        target_marker.pose.position.y = target_point.y
        target_marker.pose.position.z = 0.1  # Slightly above the ground
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.1
        target_marker.scale.y = 0.1
        target_marker.scale.z = 0.1
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0
        target_marker.lifetime.sec = 0
        target_marker.lifetime.nanosec = 100000000  # 0.1 second
        
        marker_array.markers.append(target_marker)
        self.target_point_pub.publish(marker_array)

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('path_type', 'square')  # circle, square, figure8
        self.declare_parameter('radius', 1.0)          # for circle and figure8
        self.declare_parameter('side_length', 1.25)     # for square
        self.declare_parameter('num_points', 100)      # number of points in the path
        
        # Get parameters
        self.path_type = self.get_parameter('path_type').value
        self.radius = self.get_parameter('radius').value
        self.side_length = self.get_parameter('side_length').value
        self.num_points = self.get_parameter('num_points').value
        
        # Publisher for the trajectory
        self.trajectory_pub = self.create_publisher(Path, 'trajectory', 10)
        
        # Service to generate and publish a trajectory
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.has_published = False
        
        self.get_logger().info('Trajectory generator initialized')
    
    def timer_callback(self):
        if not self.has_published:
            self.generate_and_publish_trajectory()
            self.has_published = True
    
    def generate_and_publish_trajectory(self):
        # Generate trajectory based on the specified path type
        if self.path_type == 'circle':
            trajectory = self.generate_circle_trajectory()
        elif self.path_type == 'square':
            trajectory = self.generate_square_trajectory()
        elif self.path_type == 'figure8':
            trajectory = self.generate_figure8_trajectory()
        else:
            self.get_logger().error(f'Unknown path type: {self.path_type}')
            return
        
        # Publish the trajectory
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Published {self.path_type} trajectory with {len(trajectory.poses)} points')
    
    def generate_circle_trajectory(self):
        """Generate a circular trajectory"""
        trajectory = Path()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'odom'  # Assuming odom frame
        
        for i in range(self.num_points + 1):
            angle = 2.0 * math.pi * i / self.num_points
            
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose.position.x = self.radius * math.cos(angle)
            pose.pose.position.y = self.radius * math.sin(angle)
            pose.pose.position.z = 0.0
            
            # Calculate orientation (tangent to the circle)
            yaw = angle + math.pi / 2.0  # Perpendicular to the radius
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            trajectory.poses.append(pose)
        
        return trajectory
    
    def generate_square_trajectory(self):
        """Generate a square trajectory"""
        trajectory = Path()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'odom'  # Assuming odom frame
        
        # Define the four corners of the square
        corners_old = [
            (self.side_length/2, self.side_length/2),
            (-self.side_length/2, self.side_length/2),
            (-self.side_length/2, -self.side_length/2),
            (self.side_length/2, -self.side_length/2)
        ]

        corners = [
            (0, 0),
            # (self.side_length / 3, 0),
            # (self.side_length / 3 * 2, 0),
            (-self.side_length, 0),
            (self.side_length, self.side_length),
            (0, -self.side_length)
        ]
        
        # Add the first corner again to close the square
        corners.append(corners[0])
        
        # Number of points per side
        points_per_side = self.num_points // 4
        
        for i in range(4):
            start_x, start_y = corners[i]
            end_x, end_y = corners[i+1]
            
            # Calculate orientation (tangent to the path)
            yaw = math.atan2(end_y - start_y, end_x - start_x)
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            
            for j in range(points_per_side + 1):
                t = j / points_per_side
                x = start_x + t * (end_x - start_x)
                y = start_y + t * (end_y - start_y)
                
                pose = PoseStamped()
                pose.header = trajectory.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                
                trajectory.poses.append(pose)
        
        return trajectory
    
    def generate_figure8_trajectory(self):
        """Generate a figure-8 trajectory"""
        trajectory = Path()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'odom'  # Assuming odom frame
        
        for i in range(self.num_points + 1):
            t = 2.0 * math.pi * i / self.num_points
            
            # Lemniscate of Bernoulli (figure-8 curve)
            denominator = 1.0 + math.sin(t) ** 2
            x = self.radius * math.cos(t) / denominator
            y = self.radius * math.sin(t) * math.cos(t) / denominator
            
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Calculate orientation (tangent to the curve)
            dx_dt = -self.radius * (math.sin(t) + 3 * math.sin(t) * math.cos(t) ** 2) / denominator ** 2
            dy_dt = self.radius * (math.cos(t) ** 2 - math.sin(t) ** 2) / denominator ** 2
            yaw = math.atan2(dy_dt, dx_dt)
            
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            trajectory.poses.append(pose)
        
        return trajectory
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(0.0)
        sp = math.sin(0.0)
        cr = math.cos(0.0)
        sr = math.sin(0.0)
        
        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = cy * sp * cr + sy * cp * sr
        qz = sy * cp * cr - cy * sp * sr
        
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    
    # Create the trajectory follower node
    follower_node = TrajectoryFollower()
    
    # Create the trajectory generator node
    generator_node = TrajectoryGenerator()
    
    # Set up multithreaded executor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(follower_node)
    executor.add_node(generator_node)
    
    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        executor.shutdown()
        follower_node.destroy_node()
        generator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()