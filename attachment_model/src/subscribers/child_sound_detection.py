#!/usr/bin/env python

import rospy                            # ROS Python interface
from attachment_model.msg import Care   # ROS child listener subscriber

class ChildSoundDetection(object):
    
    """
        Update with the messages of child voice detection
    """
    def __init__(self):
        self.care = False
        self.subscriber = rospy.Subscriber(
            '/child/sound_detected', Care, self.child_cb
        )

    def child_cb(self, action_data):
        self.care = action_data.initiating