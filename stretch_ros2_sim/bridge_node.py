# import math
# import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from stretch_mujoco.stretch_mujoco import StretchMujocoSimulator
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

        self.model, self.xml, self.dict = model_generation_wizard()
        self.robot_sim = StretchMujocoSimulator(model=self.model)
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

        # self.imu_mobile_base_pub = self.create_publisher(Imu, 'imu_mobile_base', 1)
        # self.magnetometer_mobile_base_pub = self.create_publisher(MagneticField, 'magnetometer_mobile_base', 1)
        # self.imu_wrist_pub = self.create_publisher(Imu, 'imu_wrist', 1)
        # self.runstop_event_pub = self.create_publisher(Bool, 'is_runstopped', 1)
        
        # self.is_gamepad_dongle_pub = self.create_publisher(Bool,'is_gamepad_dongle', 1)
        # self.gamepad_state_pub = self.create_publisher(Joy,'stretch_gamepad_state', 1) # decode using gamepad_conversion.unpack_joy_to_gamepad_state() on client side
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        # self.joint_limits_pub = self.create_publisher(JointState, 'joint_limits', 1)

        self.create_subscription(Twist, "cmd_vel", self.drive_callback, 1)
        self.create_subscription(Float64MultiArray, "joint_pose_cmd", self.joints_cmd_callback, 1)
        # self.create_subscription(Joy, "gamepad_joy", self.set_gamepad_motion_callback, 1)

        # TODO make this a config
        self.robot_mode = "gamepad"


        # Poses
        self.robot_sim.home()
        self.robot_sim.stow()

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
        # self.robot.base.translate_by(msg.data[Idx.BASE_TRANSLATE])
        # self.robot.base.rotate_by(msg.data[Idx.BASE_ROTATE])

        self.robot_sim.move_to('gripper', msg.data[Idx.GRIPPER])

    def drive_callback(self, msg : Twist):
        # Base Velocity control
        self.robot_sim.set_base_velocity(msg.linear.x, msg.angular.z)

    
    def update_simulation(self):
        # print(self.robot_sim.status)
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

        # Publish as a ROS 2 Pose message
        pose_msg = Odometry()
        pose_msg.pose = PoseWithCovariance()
        pose_msg.pose.pose = Pose()
        pose_msg.pose.pose.position.x = self.robot_sim.get_base_pose()[0]
        pose_msg.pose.pose.position.y = self.robot_sim.get_base_pose()[1]
        pose_msg.pose.pose.orientation.z = self.robot_sim.get_base_pose()[2]
        
        self.odom_pub.publish(pose_msg)


        # Get Camera Frames
        # camera_data = self.robot_sim.pull_camera_data()
        # print(camera_data)
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # Kills simulation process

if __name__ == '__main__':
    main()


