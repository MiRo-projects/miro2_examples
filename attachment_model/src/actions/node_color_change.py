#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import UInt32MultiArray

class NodeColorChange(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.color_pub = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

    # Set color format in proper form
    def convert_color(self, red = 0.0, green = 0.0, blue = 0.0):
        self.color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%self.color_detail
        color = int(color, 16)
        return color

    # Change color
    def set_color_cmd(self, color1, color2, color3, color4, color5, color6):
        self.color_change = UInt32MultiArray()
        # color = color | (int(red) << 16)
        self.color_change.data = [color1,
            color2,
            color3,
            color4,
            color5,
            color6,
        ]
        self.color_pub.publish(self.color_change)