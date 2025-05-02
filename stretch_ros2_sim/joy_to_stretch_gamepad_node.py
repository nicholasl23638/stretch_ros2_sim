# import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from rclpy.impl.logging_severity import LoggingSeverity


class Idx: # these are the ids for the actual stretch robot
    ARM = 0
    LIFT = 1
    WRIST_YAW = 2
    WRIST_PITCH = 3
    WRIST_ROLL = 4
    HEAD_PAN = 5
    HEAD_TILT = 6
    GRIPPER = 7
    BASE_TRANSLATE = 8
    BASE_ROTATE = 9

    num_joints = 10

class JoyToStretchGamepad(Node):
    def __init__(self):
        super().__init__('joy_to_stretch_gamepad_node')

        self.last_msg = Joy()
        self.last_msg.axes = [0.0] * 8
        self.last_msg.buttons = [0] * 11

        # if False, Dpad controls head, else Dpad controls wrist
        self.is_wrist_ctrl = True
        self.need_aggregator = True
        self.base_pub = Twist()
        self.joints_pub = Float64MultiArray()
        self.joints_pub.data = [0.0] * 10

        # self.create_subscription(Twist, "cmd_vel", self.drive_callback, 1)
        self.sub = self.create_subscription(Joy, '/joy', self.update_gamepad_callback, 10)
        self.get_logger().info("Subscriber for /joy topic created")

        self.base_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 1)
        self.joint_pose_pub = self.create_publisher(Float64MultiArray, '/joint_pose_cmd', 1)
        self.timer = self.create_timer(0.05, self.publish_topics)  # 20Hz control loop
    
    def update_gamepad_callback(self, msg : Joy):        
        # Buttons:
        # 0 : A
        # 1 : B
        # 2 : X
        # 3 : Y
        # 4 : LBump
        # 5 : RBump
        # 6 : LMiddleButton
        # 7 : RMiddleButton
        # 8 :  
        # 9 : LJoyPress
        # 10 : RJoyPress

        self.joints_pub = Float64MultiArray()
        self.joints_pub.data = [0.0] * 10

        if (msg.buttons[0]):
            # lift down - A
            self.joints_pub.data[Idx.LIFT] = -0.7
        elif (msg.buttons[3]):
            # lift up - Y
            self.joints_pub.data[Idx.LIFT] = 0.7

        if (msg.buttons[2]):
            # arm in / left - X
            self.joints_pub.data[Idx.ARM] = -1
        elif (msg.buttons[1]):
            # arm out / right - B
            self.joints_pub.data[Idx.ARM] = 1
        
        # update is_wrist_ctrl - RMiddle Button
        if (self.last_msg.buttons[7] != msg.buttons[7] and msg.buttons[7] == 1):
            self.is_wrist_ctrl = not self.is_wrist_ctrl
            log_msg = "Switched is_wrist_ctrl to " + str(self.is_wrist_ctrl)
            self.get_logger().log(log_msg, LoggingSeverity.INFO)

        # gripper control
        if (msg.buttons[4]):
            # grip in
            self.joints_pub.data[Idx.GRIPPER] = -0.5
        elif (msg.buttons[5]):
            # grip out
            self.joints_pub.data[Idx.GRIPPER] = 0.5
        
        # Axes:
        # 0: LJoyX
        # 1: LJoyY
        # 2: LTrigger
        # 3: RJoyX
        # 4: RJoyY
        # 5: RTrigger
        # 6: DPadX
        # 7: DPadY

        if (self.is_wrist_ctrl):
            # wrist control

            # wrist yaw
            if (msg.axes[6] > 0.0):
                self.joints_pub.data[Idx.WRIST_YAW] = 0.5
            elif (msg.axes[6] < 0.0):
                self.joints_pub.data[Idx.WRIST_YAW] = -0.5

            # wrist pitch
            if (msg.axes[7] > 0.0):
                self.joints_pub.data[Idx.WRIST_PITCH] = 0.5
            elif (msg.axes[7] < 0.0):
                self.joints_pub.data[Idx.WRIST_PITCH] = -0.5
        else:
            # head control

            # head pan
            if (msg.axes[6] > 0.0):
                self.joints_pub.data[Idx.HEAD_PAN] = 0.1
            elif (msg.axes[6] < 0.0):
                self.joints_pub.data[Idx.HEAD_PAN] = -0.1

            # head tilt
            if (msg.axes[7] > 0.0):
                self.joints_pub.data[Idx.HEAD_TILT] = 0.1
            elif (msg.axes[7] < 0.0):
                self.joints_pub.data[Idx.HEAD_TILT] = -0.1

        # self.joint_pose_pub.publish(self.joints_pub)

        self.base_pub = Twist()
        self.base_pub.linear.x = msg.axes[1] * 3
        self.base_pub.angular.z = msg.axes[3] * 3
        # self.base_vel_pub.publish(self.base_pub)

        self.last_msg = msg

    def publish_topics(self):
        # if (self.need_aggregator):
        self.base_vel_pub.publish(self.base_pub)
        self.joint_pose_pub.publish(self.joints_pub)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToStretchGamepad()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
