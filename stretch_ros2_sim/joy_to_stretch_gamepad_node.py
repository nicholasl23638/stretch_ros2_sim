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

        # self.create_subscription(Twist, "cmd_vel", self.drive_callback, 1)
        self.sub = self.create_subscription(Joy, '/joy', self.update_gamepad_callback, 10)
        self.get_logger().info("Subscriber for /joy topic created")

        self.base_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.joint_pose_pub = self.create_publisher(Float64MultiArray, 'joint_pose_cmd', 1)
    
    def update_gamepad_callback(self, msg : Joy):
        # self.get_logger().log("recieved Joy msg!", LoggingSeverity.INFO)
        if (self.last_msg.axes == msg.axes and self.last_msg.buttons == msg.buttons):
            return
        
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

        joints_pub = Float64MultiArray()
        joints_pub.data = [0.0] * 10

        if (msg.buttons[0]):
            # lift down - A
            joints_pub.data[Idx.LIFT] = -1.0
        elif (msg.buttons[3]):
            # lift up - Y
            joints_pub.data[Idx.LIFT] = 1.0

        if (msg.buttons[2]):
            # arm in / left - X
            joints_pub.data[Idx.ARM] = -1.0
        elif (msg.buttons[1]):
            # arm out / right - B
            joints_pub.data[Idx.ARM] = 1.0
        
        # update is_wrist_ctrl - RMiddle Button
        if (self.last_msg.buttons[7] != msg.buttons[7] and msg.buttons[7] == 1):
            self.is_wrist_ctrl = not self.is_wrist_ctrl
            log_msg = "Switched is_wrist_ctrl to " + str(self.is_wrist_ctrl)
            self.get_logger().log(log_msg, LoggingSeverity.INFO)

        # gripper control
        if (msg.buttons[4]):
            # grip in
            joints_pub.data[Idx.GRIPPER] = -1.0
        elif (msg.buttons[5]):
            # grip out
            joints_pub.data[Idx.GRIPPER] = 1.0
        
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
                joints_pub.data[Idx.WRIST_YAW] = 1.0
            elif (msg.axes[6] < 0.0):
                joints_pub.data[Idx.WRIST_YAW] = -1.0

            # wrist pitch
            if (msg.axes[7] > 0.0):
                joints_pub.data[Idx.WRIST_PITCH] = 1.0
            elif (msg.axes[7] < 0.0):
                joints_pub.data[Idx.WRIST_PITCH] = -1.0
        else:
            # head control

            # head pan
            if (msg.axes[6] > 0.0):
                joints_pub.data[Idx.HEAD_PAN] = 1.0
            elif (msg.axes[6] < 0.0):
                joints_pub.data[Idx.HEAD_PAN] = -1.0

            # head tilt
            if (msg.axes[7] > 0.0):
                joints_pub.data[Idx.HEAD_TILT] = 1.0
            elif (msg.axes[7] < 0.0):
                joints_pub.data[Idx.HEAD_TILT] = -1.0

        self.joint_pose_pub.publish(joints_pub)

        base_pub = Twist()
        base_pub.linear.x = msg.axes[1] * 6
        base_pub.angular.z = msg.axes[3] * 6
        self.base_vel_pub.publish(base_pub)

        self.last_msg = msg

def main(args=None):
    rclpy.init(args=args)
    node = JoyToStretchGamepad()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
