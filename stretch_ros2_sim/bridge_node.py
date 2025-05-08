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

from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.robocasa_gen import model_generation_wizard

class Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 8
    BASE_ROTATE = 9

    num_joints = 10

class StretchMujocoBridge(Node):
    def __init__(self):
        super().__init__('stretch_mujoco_bridge')
        self.img_tools = ImageTools()

        # TODO make this a config to choose vel xml
        self.mode = "pos"

        # self.model, self.xml, self.dict = model_generation_wizard()
        self.robot_sim = StretchMujocoSimulator(scene_xml_path='/home/ardenk14/ros_humble_ws/src/stretch_ros2_sim/xml/tippetop.xml')
        self.robot_sim.start()
        # robot_sim = StretchMujocoSimulator('./scene.xml')
        # robot_sim.start() # This will start the simulation and open Mujoco-Viewer window
        
        # ROS 2 Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        # self.power_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.homed_pub = self.create_publisher(Bool, 'is_homed', 1)
        self.mode_pub = self.create_publisher(String, 'mode', 1)
        # self.tool_pub = self.create_publisher(String, 'tool', 1)
        self.streaming_position_mode_pub = self.create_publisher(Bool, 'is_streaming_position', 1)
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)

        self.cam_d405_rgb_pub = self.create_publisher(Image, 'cam_d405_rgb', 1)
        self.cam_d405_depth_pub = self.create_publisher(Image, 'cam_d405_depth', 1)
        self.cam_d435i_rgb_pub = self.create_publisher(Image, 'cam_d435i_rgb', 1)
        self.cam_d435i_depth_pub = self.create_publisher(Image, 'cam_d435i_depth', 1)
        self.cam_nav_rgb_pub = self.create_publisher(Image, 'cam_nav_rgb', 1)
        self.cam_d405_K_pub = self.create_publisher(Image, 'cam_d405_K', 1)
        self.cam_d435i_K_pub = self.create_publisher(Image, 'cam_d435i_K', 1)


        self.create_subscription(Twist, "/stretch/cmd_vel", self.drive_callback, 1)
        self.create_subscription(Float64MultiArray, "/joint_pose_cmd", self.joints_cmd_callback, 1)
        # self.create_subscription(JointTrajectory, "joint_trajectory", self.trajectory_cmd_callback, 1)


        # Poses CAN'T SET WHEN USING VEL MODE
        # self.robot_sim.home()
        # self.robot_sim.stow()

        # Init conditions
        pub = Bool()
        pub.data = True
        self.homed_pub.publish(pub)
        self.streaming_position_mode_pub.publish(pub)

        pub = String()
        pub.data = "TBD"
        self.mode_pub.publish(pub)
        
        # Timer for updating the simulation
        self.timer = self.create_timer(0.01, self.update_simulation)
    
    def joints_cmd_callback(self, msg: Float64MultiArray):
        """Receives joint commands and applies them to the PyBullet object."""
        self.robot_sim.move_to("arm", msg.data[Idx.ARM])
        self.robot_sim.move_to("lift", msg.data[Idx.LIFT])
        self.robot_sim.move_to('wrist_yaw', msg.data[Idx.WRIST_YAW])
        self.robot_sim.move_to('wrist_pitch', msg.data[Idx.WRIST_PITCH])
        self.robot_sim.move_to('wrist_roll', msg.data[Idx.WRIST_ROLL])
        self.robot_sim.move_to('head_pan', msg.data[Idx.HEAD_PAN])
        self.robot_sim.move_to('head_tilt', msg.data[Idx.HEAD_TILT])
        self.robot_sim.move_to('gripper', msg.data[Idx.GRIPPER])

    def drive_callback(self, msg : Twist):
        # Base Velocity control
        self.robot_sim.set_base_velocity(msg.linear.x, msg.angular.z)

    # def trajectory_cmd_callback(self, msg : JointTrajectory):
    #     pass
    
    def update_simulation(self):
        """
        Output:
        {'time': 6.421999999999515,
        'base': {'x_vel': -3.293721562016785e-07,'theta_vel': -3.061556698064456e-05},
        'lift': {'pos': 0.5889703729548038, 'vel': 1.3548342274419937e-08},
        'arm': {'pos': 0.09806380391427844, 'vel': -0.0001650879063921366},
        'head_pan': {'pos': -4.968686850480367e-06, 'vel': 3.987855066304579e-08},
        'head_tilt': {'pos': -0.00451929555883404, 'vel': -2.2404905787897265e-09},
        'wrist_yaw': {'pos': 0.004738908190630005, 'vel': -5.8446467640096307e-05},
        'wrist_pitch': {'pos': -0.0033446975569971366,'vel': -4.3182498418896415e-06},
        'wrist_roll': {'pos': 0.0049449466225058416, 'vel': 1.27366845279872e-08},
        'gripper': {'pos': -0.00044654737698173895, 'vel': -8.808287459130369e-07}}
        """

        ### - - - - - - - - - - - - - - - - - Publish as a ROS 2 Pose message - - - - - - - - - - - - - - - - - ###
        # base_state = self.robot_sim.get_base_pose()
        # x = base_state[0]
        # y = base_state[1]
        # theta = base_state[2]
        # odom = Odometry()
        # odom.header.stamp = current_time
        # odom.header.frame_id = self.odom_frame_id
        # odom.child_frame_id = self.base_frame_id
        # odom.pose.pose.position.x = x
        # odom.pose.pose.position.y = y

        # odom.twist.twist.linear.x = x_vel
        # odom.twist.twist.linear.y = y_vel
        # odom.twist.twist.angular.z = theta_vel
        # self.odom_pub.publish(odom)

        pose_msg = Odometry()
        pose_msg.pose = PoseWithCovariance()
        pose_msg.pose.pose = Pose()
        pose_msg.pose.pose.position.x = self.robot_sim.get_base_pose()[0]
        pose_msg.pose.pose.position.y = self.robot_sim.get_base_pose()[1]

        q = quaternion_from_euler(0.0, 0.0, self.robot_sim.get_base_pose()[2])
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(pose_msg)

        ### - - - - - - - - - - - - - - - - -  Publish joint state for the arm - - - - - - - - - - - - - - - - - ###
        # formatting adapted from: /stretch_ros2/stretch_core/stretch_core/stretch_driver.py        
        joint_state = JointState()
        # joint_state.header.stamp = self.robot_sim.status.get('time') 
        # joint_arm_l3 is the most proximal and joint_arm_l0 is the
        # most distal joint of the telescoping arm model. The joints
        # are connected in series such that moving the most proximal
        # joint moves all the other joints in the global frame.
        joint_state.name = ['wrist_extension', 'joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']

        status = self.robot_sim.pull_status() # Makes this code compatible with latest updates in simulator

        arm_state = status.__getitem__('arm') # Makes this code compatible with latest updates in simulator
        pos_out = arm_state.pos
        vel_out = arm_state.vel
        eff_out = arm_state.pos # TODO PLACEHOLDER - I doubt we're gonna need this tho

        lift_state = status.__getitem__('lift')
        pos_up = lift_state.pos
        vel_up = lift_state.vel
        eff_up = lift_state.pos
        # set positions of the telescoping joints
        positions = [pos_out / 4.0 for i in range(4)]
        positions.insert(0, pos_up)
        positions.insert(0, pos_out)
        # set velocities of the telescoping joints
        velocities = [vel_out / 4.0 for i in range(4)]
        velocities.insert(0, vel_up)
        velocities.insert(0, vel_out)
        # set efforts of the telescoping joints
        efforts = [eff_out for i in range(4)]
        efforts.insert(0, eff_up)
        efforts.insert(0, eff_out)

        # if self.use_robotis_head:
        head_joint_names = ['joint_head_pan', 'joint_head_tilt']
        joint_state.name.extend(head_joint_names)
        head_pan_state = status.__getitem__('head_pan')
        head_pan_rad = head_pan_state.pos
        head_pan_vel = head_pan_state.vel
        head_pan_effort = head_pan_state.pos
        positions.append(head_pan_rad)
        velocities.append(head_pan_vel)
        efforts.append(head_pan_effort)
        head_tilt_state = status.__getitem__('head_tilt')
        head_tilt_rad = head_tilt_state.pos
        head_tilt_vel = head_tilt_state.vel
        head_tilt_effort = head_tilt_state.pos 
        positions.append(head_tilt_rad)
        velocities.append(head_tilt_vel)
        efforts.append(head_tilt_effort)

        # if self.use_robotis_end_of_arm:
        end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        end_of_arm_joint_names = end_of_arm_joint_names + ['joint_gripper_finger_left', 'joint_gripper_finger_right']
        joint_state.name.extend(end_of_arm_joint_names)
        wrist_yaw_state = status.__getitem__('wrist_yaw')
        wrist_yaw_rad = wrist_yaw_state.pos
        wrist_yaw_vel = wrist_yaw_state.vel
        wrist_yaw_effort = wrist_yaw_state.pos
        positions.append(wrist_yaw_rad)
        velocities.append(wrist_yaw_vel)
        efforts.append(wrist_yaw_effort)

        # if dex_wrist_attached:
        wrist_pitch_state = status.__getitem__('wrist_pitch')
        wrist_pitch_rad = wrist_pitch_state.pos
        wrist_pitch_vel = wrist_pitch_state.vel
        wrist_pitch_effort = wrist_pitch_state.pos
        positions.append(wrist_pitch_rad)
        velocities.append(wrist_pitch_vel)
        efforts.append(wrist_pitch_effort)
        wrist_roll_state = status.__getitem__('wrist_roll')
        wrist_roll_rad = wrist_roll_state.pos
        wrist_roll_vel = wrist_roll_state.vel
        wrist_roll_effort = wrist_roll_state.pos 
        positions.append(wrist_roll_rad)
        velocities.append(wrist_roll_vel)
        efforts.append(wrist_roll_effort)

        # if 'stretch_gripper' in self.robot.end_of_arm.joints:
        gripper_state = status.__getitem__('gripper')
        gripper_finger_rad = gripper_state.pos
        gripper_finger_vel = gripper_state.vel
        gripper_finger_effort = gripper_state.pos
        positions.append(gripper_finger_rad)
        velocities.append(gripper_finger_vel)
        efforts.append(gripper_finger_effort)
        positions.append(gripper_finger_rad)
        velocities.append(gripper_finger_vel)
        efforts.append(gripper_finger_effort)

        # Set joint state
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_pub.publish(joint_state)


        ### - - - - - - - - - - - - - - - - -  Publish Camera Frames - - - - - - - - - - - - - - - - - ###
        # np.set_printoptions(threshold=sys.maxsize)
        camera_data = self.robot_sim.pull_camera_data()
        pub_data = Image()
        cam_d405_rgb = np.array(camera_data.cam_d405_rgb)
        try:
            pub_data = self.img_tools.convert_cv2_to_ros_msg(cam_d405_rgb)
            # self.get_logger().log(str(pub_data), LoggingSeverity.INFO)
            self.cam_d405_rgb_pub.publish(pub_data)
        except:
            pass

        # pub_data.data = np.array(camera_data.get('cam_d405_depth')).flatten()
        # self.cam_d405_depth_pub.publish(pub_data)
        # pub_data.data = np.array(camera_data.get('cam_d435i_rgb')).flatten()
        # self.cam_d435i_rgb_pub.publish(pub_data)
        # pub_data.data = np.array(camera_data.get('cam_d435i_depth')).flatten()
        # self.cam_d435i_depth_pub.publish(pub_data)
        # pub_data.data = np.array(camera_data.get('cam_nav_rgb')).flatten()
        # self.cam_nav_rgb_pub.publish(pub_data)
        # pub_data.data = np.array(camera_data.get('cam_d405_K')).flatten()
        # self.cam_d405_K_pub.publish(pub_data)
        # pub_data.data = np.array(camera_data.get('cam_d435i_K')).flatten()
        # self.cam_d435i_K_pub.publish(pub_data)
        """
        Output:
        {'time': 80.89999999999286,
        'cam_d405_rgb': array([[...]]),
        'cam_d405_depth': array([[...]]),
        'cam_d435i_rgb': array([[...]]),
        'cam_d435i_depth': array([[...]]),
        'cam_nav_rgb': array([[...]]),
        'cam_d405_K': array([[...]]),
        'cam_d435i_K': array([[...]])}
        """
                

    def destroy_node(self):
        self.robot_sim.stop()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StretchMujocoBridge()
    # try:
    rclpy.spin(node)
    # except:
    #     node.destroy_node()
    #     rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    # Kills simulation process

if __name__ == '__main__':
    main()

"""
ROS ImageTools, a class that contains methods to transform
from & to ROS Image, ROS CompressedImage & numpy.ndarray (cv2 image).
Also deals with Depth images, with a tricky catch, as they are compressed in
PNG, and we are here hardcoding to compression level 3 and the default
quantizations of the plugin. (What we use in our robots).

Meanwhile image_transport has no Python interface, this is the best we can do.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class ImageTools(object):
    def __init__(self):
        self._cv_bridge = CvBridge()

    def convert_ros_msg_to_cv2(self, ros_data, image_encoding='bgr8'):
        """
        Convert from a ROS Image message to a cv2 image.
        """
        try:
            return self._cv_bridge.imgmsg_to_cv2(ros_data, image_encoding)
        except CvBridgeError as e:
            if "[16UC1] is not a color format" in str(e):
                raise CvBridgeError(
                    "You may be trying to use a Image method " +
                    "(Subscriber, Publisher, conversion) on a depth image" +
                    " message. Original exception: " + str(e))
            raise e

    def convert_cv2_to_ros_msg(self, cv2_data, image_encoding='bgr8'):
        """
        Convert from a cv2 image to a ROS Image message.
        """
        return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)
    
# the rest of this file isn't needed ^ so it's been removed