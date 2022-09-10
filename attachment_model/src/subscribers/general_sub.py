#!/usr/bin/env python

# import core modules
import os
import rospy
from math import degrees
from tf.transformations import euler_from_quaternion

# import new messages
from nav_msgs.msg import Odometry   # ROS odometry subsciriber
from sensor_msgs.msg import Range   # ROS range subscriber

class GeneralSub(object):
    
    def __init__(self):

        # for odom
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0

        # for range
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0

        # for odometry data
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/odom", Odometry, self.odom_cb)
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/sonar", Range, self.range_cb)

    """
        Update the odometry
    """
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        if yaw<0:
            self.yaw = self.round(degrees(yaw), 4)+360
        else:
            self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    """
        Update the range
    """
    def range_cb(self, odom_data):
        self.field_of_view = odom_data.field_of_view
        self.min_range = odom_data.min_range
        self.max_range = odom_data.max_range
        self.range = odom_data.range

    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)