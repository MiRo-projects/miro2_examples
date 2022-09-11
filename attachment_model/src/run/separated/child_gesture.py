#!/usr/bin/env python

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

from actions.gestures_pub import ChildGestures

import rospy

class GesturesPub(object):
    
    def __init__(self):
        # rgb
        rospy.init_node("child_gesture")
        self.gesture = ChildGestures()

    def run(self):
        while not rospy.is_shutdown():
            self.gesture.change_color()
        
if __name__ == "__main__":
    gestures = GesturesPub()
    gestures.run()