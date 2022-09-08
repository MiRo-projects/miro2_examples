#!/usr/bin/env python

import rospy                            # ROS Python interface
from attachment_model.msg import Care   # ROS parent listener subscriber

class ParentListner(object):
    
    """
        Update with the messages of parent voice detection
    """
    def __init__(self):
        self.care = False
        self.subscriber = rospy.Subscriber(
            '/parent/care_detection', Care, self.parent_cb
        )

    def parent_cb(self, action_data):
        self.care = action_data.initiating