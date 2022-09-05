#!/usr/bin/env python3

import miro_actions
from sensor_audio import SensorAudio
import rospy

class AudioDemo:
    
    def __init__(self):
        rospy.init_node("Demo")
        self.audio_detection = SensorAudio()
        self.rate = rospy.Rate(500)

    def demo(self):
        while not rospy.core.is_shutdown():
            detected_word = self.audio_detection.detected_word
            if detected_word == "Good Boy":
                action = miro_actions.GoodBoy()
            elif detected_word == "Hey Miro":
                action = miro_actions.HeyMiro()
            elif detected_word == "No Miro":
                action = miro_actions.NoMiro()
            
            if detected_word != "":
                action.execute()
            self.rate.sleep()

if __name__ == "__main__":

	main = AudioDemo()
	main.demo()