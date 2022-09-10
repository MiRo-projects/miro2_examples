#!/usr/bin/env python
import rospy    # ROS Python interface
from attachment_model.msg import Action # ROS action subscriber

class ActionSub(object):
    
    def __init__(self):
        self.parent = 0
        self.child = 0
        self.emotional_distance = 0
        self.physical_distance = 0
        self.child_need = 0
        self.parent_need = 0

    def action_cb(self, action_data):
        self.parent= action_data.parent
        self.child = action_data.child
        self.emotional_distance = action_data.emotional_distance
        self.physical_distance = action_data.physical_distance
        self.child_need = action_data.child_need
        self.parent_need = action_data.parent_need

class BothAction(ActionSub):

    def __init__(self):
        self.subscriber = rospy.Subscriber("/simultaneous_controller", Action, self.action_cb)
        super().__init__()

class ChildAction(ActionSub):

    def __init__(self):
        self.subscriber = rospy.Subscriber("/child/controller", Action, self.action_cb)
        super().__init__()

class ParentAction(ActionSub):

    def __init__(self):
        self.subscriber = rospy.Subscriber("/parent/controller", Action, self.action_cb)
        super().__init__()