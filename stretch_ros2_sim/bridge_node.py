# import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy, Image
from std_msgs.msg import Bool, String, Float64MultiArray
from rclpy.impl.logging_severity import LoggingSeverity
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

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
        self.img_tools = ImageTools()

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

        self.cam_d405_rgb_pub = self.create_publisher(Image, 'cam_d405_rgb', 1)
        self.cam_d405_depth_pub = self.create_publisher(Image, 'cam_d405_depth', 1)
        self.cam_d435i_rgb_pub = self.create_publisher(Image, 'cam_d435i_rgb', 1)
        self.cam_d435i_depth_pub = self.create_publisher(Image, 'cam_d435i_depth', 1)
        self.cam_nav_rgb_pub = self.create_publisher(Image, 'cam_nav_rgb', 1)
        self.cam_d405_K_pub = self.create_publisher(Image, 'cam_d405_K', 1)
        self.cam_d435i_K_pub = self.create_publisher(Image, 'cam_d435i_K', 1)

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
        self.robot_sim.set_velocity
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

        ### - - - - - - - - - - - - - - - - - Publish as a ROS 2 Pose message - - - - - - - - - - - - - - - - - ###
        pose_msg = Odometry()
        pose_msg.pose = PoseWithCovariance()
        pose_msg.pose.pose = Pose()
        pose_msg.pose.pose.position.x = self.robot_sim.get_base_pose()[0]
        pose_msg.pose.pose.position.y = self.robot_sim.get_base_pose()[1]
        pose_msg.pose.pose.orientation.z = self.robot_sim.get_base_pose()[2]
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

        arm_state = self.robot_sim.status.get('arm')
        pos_out = arm_state.get('pos')
        vel_out = arm_state.get('vel')
        eff_out = arm_state.get('pos') # TODO PLACEHOLDER - I doubt we're gonna need this tho

        lift_state = self.robot_sim.status.get('lift')
        pos_up = lift_state.get('pos')
        vel_up = lift_state.get('vel')
        eff_up = lift_state.get('pos')
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
        head_pan_state = self.robot_sim.status.get('head_pan')
        head_pan_rad = head_pan_state.get('pos')
        head_pan_vel = head_pan_state.get('vel')
        head_pan_effort = head_pan_state.get('pos') 
        positions.append(head_pan_rad)
        velocities.append(head_pan_vel)
        efforts.append(head_pan_effort)
        head_tilt_state = self.robot_sim.status.get('head_tilt')
        head_tilt_rad = head_tilt_state.get('pos')
        head_tilt_vel = head_tilt_state.get('vel')
        head_tilt_effort = head_tilt_state.get('pos') 
        positions.append(head_tilt_rad)
        velocities.append(head_tilt_vel)
        efforts.append(head_tilt_effort)

        # if self.use_robotis_end_of_arm:
        end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        end_of_arm_joint_names = end_of_arm_joint_names + ['joint_gripper_finger_left', 'joint_gripper_finger_right']
        joint_state.name.extend(end_of_arm_joint_names)
        wrist_yaw_state = self.robot_sim.status.get('wrist_yaw')
        wrist_yaw_rad = wrist_yaw_state.get('pos')
        wrist_yaw_vel = wrist_yaw_state.get('vel')
        wrist_yaw_effort = wrist_yaw_state.get('pos') 
        positions.append(wrist_yaw_rad)
        velocities.append(wrist_yaw_vel)
        efforts.append(wrist_yaw_effort)

        # if dex_wrist_attached:
        wrist_pitch_state = self.robot_sim.status.get('wrist_pitch')
        wrist_pitch_rad = wrist_pitch_state.get('pos')
        wrist_pitch_vel = wrist_pitch_state.get('vel')
        wrist_pitch_effort = wrist_pitch_state.get('pos') 
        positions.append(wrist_pitch_rad)
        velocities.append(wrist_pitch_vel)
        efforts.append(wrist_pitch_effort)
        wrist_roll_state = self.robot_sim.status.get('wrist_roll')
        wrist_roll_rad = wrist_roll_state.get('pos')
        wrist_roll_vel = wrist_roll_state.get('vel')
        wrist_roll_effort = wrist_roll_state.get('pos') 
        positions.append(wrist_roll_rad)
        velocities.append(wrist_roll_vel)
        efforts.append(wrist_roll_effort)

        # if 'stretch_gripper' in self.robot.end_of_arm.joints:
        gripper_state = self.robot_sim.status.get('gripper')
        gripper_finger_rad = gripper_state.get('pos')
        gripper_finger_vel = gripper_state.get('vel')
        gripper_finger_effort = gripper_state.get('pos') 
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
        cam_d405_rgb = np.array(camera_data.get('cam_d405_rgb'))
        pub_data = self.img_tools.convert_cv2_to_ros_msg(cam_d405_rgb)
        # self.get_logger().log(str(pub_data), LoggingSeverity.INFO)
        self.cam_d405_rgb_pub.publish(pub_data)

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

    def convert_ros_compressed_to_cv2(self, compressed_msg):
        np_arr = np.fromstring(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def convert_ros_compressed_msg_to_ros_msg(self, compressed_msg,
                                              encoding='bgr8'):
        cv2_img = self.convert_ros_compressed_to_cv2(compressed_msg)
        ros_img = self._cv_bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
        ros_img.header = compressed_msg.header
        return ros_img

    def convert_cv2_to_ros_msg(self, cv2_data, image_encoding='bgr8'):
        """
        Convert from a cv2 image to a ROS Image message.
        """
        return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)

    def convert_cv2_to_ros_compressed_msg(self, cv2_data,
                                          compressed_format='jpg'):
        """
        Convert from cv2 image to ROS CompressedImage.
        """
        return self._cv_bridge.cv2_to_compressed_imgmsg(cv2_data,
                                                        dst_format=compressed_format)

    def convert_ros_msg_to_ros_compressed_msg(self, image,
                                              image_encoding='bgr8',
                                              compressed_format="jpg"):
        """
        Convert from ROS Image message to ROS CompressedImage.
        """
        cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding)
        cimg_msg = self._cv_bridge.cv2_to_compressed_imgmsg(cv2_img,
                                                            dst_format=compressed_format)
        cimg_msg.header = image.header
        return cimg_msg

    def convert_to_cv2(self, image):
        """
        Convert any kind of image to cv2.
        """
        cv2_img = None
        if type(image) == np.ndarray:
            cv2_img = image
        elif image._type == 'sensor_msgs/Image':
            cv2_img = self.convert_ros_msg_to_cv2(image)
        elif image._type == 'sensor_msgs/CompressedImage':
            cv2_img = self.convert_ros_compressed_to_cv2(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return cv2_img

    def convert_to_ros_msg(self, image):
        """
        Convert any kind of image to ROS Image.
        """
        ros_msg = None
        if type(image) == np.ndarray:
            ros_msg = self.convert_cv2_to_ros_msg(image)
        elif image._type == 'sensor_msgs/Image':
            ros_msg = image
        elif image._type == 'sensor_msgs/CompressedImage':
            ros_msg = self.convert_ros_compressed_msg_to_ros_msg(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_msg

    def convert_to_ros_compressed_msg(self, image, compressed_format='jpg'):
        """
        Convert any kind of image to ROS Compressed Image.
        """
        ros_cmp = None
        if type(image) == np.ndarray:
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(
                image, compressed_format=compressed_format)
        elif image._type == 'sensor_msgs/Image':
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(
                image, compressed_format=compressed_format)
        elif image._type == 'sensor_msgs/CompressedImage':
            ros_cmp = image
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_cmp

    def convert_depth_to_ros_msg(self, image):
        ros_msg = None
        if type(image) == np.ndarray:
            ros_msg = self.convert_cv2_to_ros_msg(image,
                                                  image_encoding='mono16')
        elif image._type == 'sensor_msgs/Image':
            image.encoding = '16UC1'
            ros_msg = image
        elif image._type == 'sensor_msgs/CompressedImage':
            ros_msg = self.convert_compressedDepth_to_image_msg(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_msg

    def convert_depth_to_ros_compressed_msg(self, image):
        ros_cmp = None
        if type(image) == np.ndarray:
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(image,
                                                             compressed_format='png')
            ros_cmp.format = '16UC1; compressedDepth'
            # This is a header ROS depth CompressedImage have, necessary
            # for viewer tools to see the image
            # extracted from a real image from a robot
            # The code that does it in C++ is this:
            # https://github.com/ros-perception/image_transport_plugins/blob/indigo-devel/compressed_depth_image_transport/src/codec.cpp
            ros_cmp.data = "\x00\x00\x00\x00\x88\x9c\x5c\xaa\x00\x40\x4b\xb7" + ros_cmp.data
        elif image._type == 'sensor_msgs/Image':
            image.encoding = "mono16"
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(
                image,
                image_encoding='mono16',
                compressed_format='png')
            ros_cmp.format = '16UC1; compressedDepth'
            ros_cmp.data = "\x00\x00\x00\x00\x88\x9c\x5c\xaa\x00\x40\x4b\xb7" + ros_cmp.data
        elif image._type == 'sensor_msgs/CompressedImage':
            ros_cmp = image
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_cmp

    def convert_depth_to_cv2(self, image):
        cv2_img = None
        if type(image) == np.ndarray:
            cv2_img = image
        elif image._type == 'sensor_msgs/Image':
            image.encoding = 'mono16'
            cv2_img = self.convert_ros_msg_to_cv2(image,
                                                  image_encoding='mono16')
        elif image._type == 'sensor_msgs/CompressedImage':
            cv2_img = self.convert_compressedDepth_to_cv2(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return cv2_img

    def convert_compressedDepth_to_image_msg(self, compressed_image):
        """
        Convert a compressedDepth topic image into a ROS Image message.
        compressed_image must be from a topic /bla/compressedDepth
        as it's encoded in PNG
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        depth_img_raw = self.convert_compressedDepth_to_cv2(compressed_image)
        img_msg = self._cv_bridge.cv2_to_imgmsg(depth_img_raw, "mono16")
        img_msg.header = compressed_image.header
        img_msg.encoding = "16UC1"
        return img_msg

    def convert_compressedDepth_to_cv2(self, compressed_depth):
        """
        Convert a compressedDepth topic image into a cv2 image.
        compressed_depth must be from a topic /bla/compressedDepth
        as it's encoded in PNG
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        depth_fmt, compr_type = compressed_depth.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")

        # remove header from raw data, if necessary
        if 'PNG' in compressed_depth.data[:12]:
            # If we compressed it with opencv, there is nothing to strip
            depth_header_size = 0
        else:
            # If it comes from a robot/sensor, it has 12 useless bytes apparently
            depth_header_size = 12
        raw_data = compressed_depth.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8),
                                     # the cv2.CV_LOAD_IMAGE_UNCHANGED has been removed
                                     -1)  # cv2.CV_LOAD_IMAGE_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")
        return depth_img_raw

    def display_image(self, image):
        """
        Use cv2 to show an image.
        """
        cv2_img = self.convert_to_cv2(image)
        window_name = 'show_image press q to exit'
        cv2.imshow(window_name, cv2_img)
        # TODO: Figure out how to check if the window
        # was closed... when a user does it, the program is stuck
        key = cv2.waitKey(0)
        if chr(key) == 'q':
            cv2.destroyWindow(window_name)

    def save_image(self, image, filename):
        """
        Given an image in numpy array or ROS format
        save it using cv2 to the filename. The extension
        declares the type of image (e.g. .jpg .png).
        """
        cv2_img = self.convert_to_cv2(image)
        cv2.imwrite(filename, cv2_img)

    def save_depth_image(self, image, filename):
        """
        Save a normalized (easier to visualize) version
        of a depth image into a file.
        """
        # Duc's smart undocummented code
        im_array = self.convert_depth_to_cv2(image)
        min_distance, max_distance = np.min(im_array), np.max(im_array)
        im_array = im_array * 1.0
        im_array = (im_array < max_distance) * im_array
        im_array = (im_array - min_distance) / max_distance * 255.0
        im_array = (im_array >= 0) * im_array

        cv2.imwrite(filename, im_array)

    def load_from_file(self, file_path, cv2_imread_mode=None):
        """
        Load image from a file.
        :param file_path str: Path to the image file.
        :param cv2_imread_mode int: cv2.IMREAD_ mode, modes are:
            cv2.IMREAD_ANYCOLOR 4             cv2.IMREAD_REDUCED_COLOR_4 33
            cv2.IMREAD_ANYDEPTH 2             cv2.IMREAD_REDUCED_COLOR_8 65
            cv2.IMREAD_COLOR 1                cv2.IMREAD_REDUCED_GRAYSCALE_2 16
            cv2.IMREAD_GRAYSCALE 0            cv2.IMREAD_REDUCED_GRAYSCALE_4 32
            cv2.IMREAD_IGNORE_ORIENTATION 128 cv2.IMREAD_REDUCED_GRAYSCALE_8 64
            cv2.IMREAD_LOAD_GDAL 8            cv2.IMREAD_UNCHANGED -1
            cv2.IMREAD_REDUCED_COLOR_2 17
        """
        if cv2_imread_mode is not None:
            img = cv2.imread(file_path, cv2_imread_mode)
        img = cv2.imread(file_path)
        if img is None:
            raise RuntimeError("No image found to load at " + str(file_path))
        return img


if __name__ == '__main__':
    from sensor_msgs.msg import Image, CompressedImage
    it = ImageTools()
    from cPickle import load
    img = load(open('rgb_image.pickle', 'r'))
    cv2_img = it.convert_ros_msg_to_cv2(img)
    print(type(cv2_img))
    ros_img = it.convert_cv2_to_ros_msg(cv2_img)
    print(type(ros_img))
    ros_compressed2 = it.convert_cv2_to_ros_compressed_msg(cv2_img)
    print(type(ros_compressed2))
    compressed_ros_img = it.convert_ros_msg_to_ros_compressed_msg(ros_img)
    print(type(compressed_ros_img))
    ros_img2 = it.convert_ros_compressed_msg_to_ros_msg(compressed_ros_img)
    print(type(ros_img2))
    cv2_2 = it.convert_ros_compressed_to_cv2(compressed_ros_img)
    print(type(cv2_2))

    cv2_3 = it.convert_to_cv2(cv2_img)
    cv2_4 = it.convert_to_cv2(ros_img)
    cv2_5 = it.convert_to_cv2(compressed_ros_img)

    ros_3 = it.convert_to_ros_msg(cv2_img)
    ros_4 = it.convert_to_ros_msg(ros_img)
    ros_5 = it.convert_to_ros_msg(compressed_ros_img)

    rosc_3 = it.convert_to_ros_compressed_msg(cv2_img)
    rosc_4 = it.convert_to_ros_compressed_msg(ros_img)
    rosc_5 = it.convert_to_ros_compressed_msg(compressed_ros_img)

    depthcompimg = load(open('depth_compressed_image.pickle', 'r'))
    depth_cv2 = it.convert_depth_to_cv2(depthcompimg)
    depth_ros = it.convert_depth_to_ros_msg(depthcompimg)
    depth_ros_comp = it.convert_depth_to_ros_compressed_msg(depthcompimg)
    it.save_image(depth_cv2, 'depth_comp_cv2.jpg')
    it.save_depth_image(depth_cv2, 'depth_comp_normalized_cv2.jpg')

    depthimg = load(open('depth_image.pickle', 'r'))
    depthimg_cv2 = it.convert_depth_to_cv2(depthimg)
    depthimg_ros = it.convert_depth_to_ros_msg(depthimg)
    depthimg_ros_comp = it.convert_depth_to_ros_compressed_msg(depthimg)
    it.save_image(depth_cv2, 'depth_cv2.jpg')
    it.save_depth_image(depth_cv2, 'depth_normalized_cv2.jpg')

    it.save_image(cv2_img, 'cv2_image.jpg')
    it.save_image(ros_img, 'ros_image.jpg')
    it.save_image(compressed_ros_img, 'compressed_ros_image.jpg')

    it.display_image(cv2_img)
