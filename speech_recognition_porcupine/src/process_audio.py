#!/user/bin/env python3
import numpy as np
import os
import rospy # ROS Python interface
from std_msgs.msg import Int16MultiArray # ROS message for mics

class ProcessAudio(object):
    
    # update the detected words with the messages being published
    def __init__(self):
        self.mics = []
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            "/miro/sensors/mics", Int16MultiArray, self.processed_audio_cb
        )

    def processed_audio_cb(self, action_data):
        self.mics = action_data.data