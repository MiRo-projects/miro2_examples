#!/usr/bin/env python

# import core modules
import os
import rospy

# import required messages
from std_msgs.msg import UInt32MultiArray   # robot illumination
from std_msgs.msg import Float32MultiArray  # cosmetic joints
from geometry_msgs.msg import TwistStamped  # odometry

"""
    Helper class to publish control over the robot
"""
class MiRoAction(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # control miro color
        self.color_pub = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

        # control cosmetic joints
        self.cos_joint_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        # control movement
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

    """
        Changes the color of the MiRo's illumination in rgb format
    """
    def set_color_cmd(self, red = 0.0, green = 0.0, blue = 0.0):
        color_change = UInt32MultiArray()
        color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%color_detail
        rospy.loginfo('Color ' + color)
        color = int(color, 16)
        color_change.data = [color,
            color,
            color,
            color,
            color,
            color,
        self.color_pub.publish(color_change)
        ]

    """
        Changes the position of the cosmetic joints
            droop => head height
            wag => tail position
            eyel => eye lids left
            eyer => eye lids right
            earl => ear twisting left
            earr => ear twisting right
    """
    def set_cos_joint(self, droop=0.0, wag=0.0, eyel=0.0, eyer=0.0, earl=0.0, earr=0.0):
        wag_tail = Float32MultiArray()
        wag_detail = [droop, wag, eyel, eyer, earl, earr]
        wag_tail.data = wag_detail
        self.cos_joint_pub.publish(wag_tail)
    
    """
        Changes the speed at the robot should travel
            linear => speed moving forward/backward
            angular => angular velocity
    """
    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = linear
        vel_cmd.twist.angular.z = angular
        self.vel_pub.publish(vel_cmd)
