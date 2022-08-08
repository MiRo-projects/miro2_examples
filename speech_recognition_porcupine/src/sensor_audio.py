#!/user/bin/env python3
import os
import rospy # ROS Python interface
from speech_recognition_porcupine.msg import Audio # ROS Audio info subscriber

class SensorAudio(object):
    
    # update the detected words with the messages being published
    def __init__(self):
        self.detected_word = ""
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            self.topic_base_name + "/audio/detected_words", Audio, self.audio_cb
        )

    def audio_cb(self, action_data):
        self.detected_word = action_data.detected_words