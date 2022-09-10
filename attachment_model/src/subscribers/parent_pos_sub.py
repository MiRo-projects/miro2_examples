#!/usr/bin/env python

import rospy                                # ROS Python interface
from attachment_model.msg import RobotPub   # ROS parent position message

class ParentPosition(object):
    
    """
        Update with messages of parent position
    """
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.subscriber = rospy.Subscriber(
            "/parent/odom_position", RobotPub, self.parent_cb
        )

    def parent_cb(self, action_data):
        self.pos_x = action_data.pos_x
        self.pos_y = action_data.pos_y