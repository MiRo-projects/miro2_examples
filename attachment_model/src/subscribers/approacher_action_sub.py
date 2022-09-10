#!/usr/bin/env python

import rospy                            # ROS Python interface
from attachment_model.msg import Care   # ROS child listener subscriber

class ChildApproacherAction(object):
    
    """
        Update with the messages of child voice detection
    """
    def __init__(self):
        self.care = False
        self.subscriber = rospy.Subscriber(
            '/child/approacher_action', Care, self.child_cb
        )

    def child_cb(self, action_data):
        self.care = action_data.initiating

class ParentApproacherAction(object):
    
    """
        Update with the messages of child voice detection
    """
    def __init__(self):
        self.care = False
        self.subscriber = rospy.Subscriber(
            '/parent/approacher_action', Care, self.child_cb
        )

    def child_cb(self, action_data):
        self.care = action_data.initiating