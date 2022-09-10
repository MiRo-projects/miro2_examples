#!/usr/bin/env python

import rospy                                # ROS Python interface
from attachment_model.msg import RobotPub   # ROS child info subscriber

class ChildPosition(object):
    
    """
        Update with the messages of child position
    """
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.subscriber = rospy.Subscriber(
            "/child/odom_position", RobotPub, self.child_cb
        )

    def child_cb(self, action_data):
        self.pos_x = action_data.pos_x
        self.pos_y = action_data.pos_y