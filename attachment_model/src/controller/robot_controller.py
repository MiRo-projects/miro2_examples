#!/usr/bin/env python3

# import core modules
from abc import abstractmethod
import rospy
import numpy as np

# import required controller modules
from controller.emotion_controller import BothEmotionController, ParentEmotionController, ChildEmotionController
from controller.physical_controller import BothPhysicalController, ParentPhysicalController, ChildPhysicalController

# import required messages
from attachment_model.msg import Action

"""
    This controller can either be independent or interacting with one another
"""
CONSTANT_A = 1.0
CONSTANT_B = 0.5
TIME_H_STEP = 0.01

class RobotController(object):
    def __init__(self, ambivalence, avoidance):

        # set emotion
        self.ambivalence = ambivalence
        self.avoidance = avoidance

        # Set rate for rospy
        self.rate = rospy.Rate(30)

        # initialise action publisher
        self.action = Action()

        # time step for model calculation
        self.time = 0

        # x is needs, y is accumulated needs, 1 is child, 2 is parent
        self.dx1 = -1.0
        self.dy1 = 1.0
        self.dx2 = -1.0
        self.dy2 = 1.0

        # setting ambivalence and avoidance
        self.epsilonAv = avoidance
        self.epsilonAm = ambivalence

        self.dp = 0.0
        self.de = 0.0

    """
        Update the ROS messages, that is the physical distance, emotional distance and the needs for parent and child
    """
    @ abstractmethod
    def update_messages(self, child_action = None, parent_action = None, child_need = None, parent_need = None):
        pass

    """
        Update the distances
    """
    @abstractmethod
    def update_distance(self):
        pass

    def central_control(self):
        # lambda function to decide whether to approach or explore
        A_explore = lambda x: np.heaviside(-x,0)

        while not rospy.is_shutdown():
            self.time += TIME_H_STEP

            # set the avoidant and ambivalent
            self.epsilonAv = self.avoidance
            self.epsilonAm = self.ambivalence
            
            # update physical and emotional distance
            self.update_distance()

            calculated_need_accumulation = [self.dx1, self.dy1, self.dx2, self.dy2]
            
            # Calculate needs and accumulated needs
            k1 = self.f(self.time, calculated_need_accumulation)
            k2 = self.f(self.time + TIME_H_STEP/2.0, calculated_need_accumulation + TIME_H_STEP*k1/2.0)
            k3 = self.f(self.time + TIME_H_STEP/2.0, calculated_need_accumulation + TIME_H_STEP*k2/2.0)
            k4 = self.f(self.time + TIME_H_STEP, calculated_need_accumulation + TIME_H_STEP*k3)
            calculated_need_accumulation = calculated_need_accumulation + TIME_H_STEP*(k1 + 2*k2 + 2*k3 + k4)/6.0

            # update messages
            self.update_messages(int(A_explore(calculated_need_accumulation[0])), int(A_explore(calculated_need_accumulation[2])), calculated_need_accumulation[0], calculated_need_accumulation[2])

            # update the needs and accumulated needs
            [self.dx1,self.dy1,self.dx2,self.dy2] = calculated_need_accumulation
            print(calculated_need_accumulation[0], calculated_need_accumulation[2], self.action.child, self.action.parent)
            self.rate.sleep()

    """
        Model
    """
    def f(self, t, r):
        x1 = r[0]   # need change 1
        y1 = r[1]   # need accumulated 1
        x2 = r[2]   # need change 2
        y2 = r[3]   # need accumulated 2
        # Calculate needs and accumulated neds
        dx1 = -CONSTANT_A*(2.0*x1**3 - x1) - y1 
        dy1 = CONSTANT_B*x1- self.epsilonAm*(self.dp + y1)  - self.epsilonAv*(self.de + y1 ) 
        dx2 = -CONSTANT_A*(2.0*x2**3 - x2) - y2
        dy2 = CONSTANT_B*x2  + self.epsilonAm*(self.dp + y2)  - self.epsilonAv*(self.de + y2)
        return np.array([dx1, dy1, dx2, dy2])

"""
    Calculate action for both parent and child
"""
class BothController(RobotController):

    def __init__(self, ambivalence, avoidance):
        # initialise objects and variables for physical distance and emotional distance calculation
        self.physical_distance_controller = BothPhysicalController()
        self.emotional_distance_controller = BothEmotionController()
        self.emotional_distance = self.emotional_distance_controller.emotional_distance()
        self.physical_distance = self.physical_distance_controller.physical_distance()

        # action message
        self.action.child = 0
        self.action.parent = 0
        self.action.emotional_distance = 0
        self.action.physical_distance = 0
        self.action.child_need = 0
        self.action.parent_need = 0

        # initialise publisher
        self.controller_pub = rospy.Publisher(
            '/simultaneous/controller', Action, queue_size= 0
        )

        super().__init__(ambivalence, avoidance)

    def update_messages(self, child_action = None, parent_action = None, child_need = None, parent_need = None):
        # set messages
        self.action.child = child_action
        self.action.parent = parent_action
        self.action.child_need = child_need
        self.action.parent_need = parent_need
        self.action.physical_distance = self.dp
        self.action.emotional_distance = self.de

        # publish
        self.controller_pub.publish(self.action)

    def update_distance(self):
        self.de = self.emotional_distance_controller.emotional_distance()
        self.dp = self.physical_distance_controller.physical_distance()

"""
    Calculate action for both child
"""
class ChildController(RobotController):

    def __init__(self, ambivalence, avoidance):
        super().__init__(ambivalence, avoidance)
        # initialise objects and variables for physical distance and emotional distance calculation
        self.physical_distance_controller = ParentPhysicalController()
        self.emotional_distance_controller = ParentEmotionController()
        self.de = self.emotional_distance_controller.emotional_distance()
        self.dp = self.physical_distance_controller.physical_distance()

        # action message
        self.action.child = 0
        self.action.parent = None
        self.action.emotional_distance = 0
        self.action.physical_distance = 0
        self.action.child_need = 0
        self.action.parent_need = None

        # initialise publisher
        self.controller_pub = rospy.Publisher(
            '/child/controller', Action, queue_size= 0
        )

    def update_messages(self, child_action = None, parent_action = None, child_need = None, parent_need = None):
        # set messages
        self.action.child = child_action
        self.action.child_need = child_need
        self.action.physical_distance = self.dp
        self.action.emotional_distance = self.de

        # publish
        self.controller_pub.publish(self.action)

    def update_distance(self):
        self.de = self.emotional_distance_controller.emotional_distance()
        self.dp = self.physical_distance_controller.physical_distance()

"""
    Calculate action for both child
"""
class ParentController(RobotController):

    def __init__(self, ambivalence, avoidance):
        super().__init__(ambivalence, avoidance)
        # initialise objects and variables for physical distance and emotional distance calculation
        self.physical_distance_controller = ChildPhysicalController()
        self.emotional_distance_controller = ChildEmotionController()
        self.de = self.emotional_distance_controller.emotional_distance()
        self.dp = self.physical_distance_controller.physical_distance()

        # action message
        self.action.child = None
        self.action.parent = 0
        self.action.emotional_distance = 0
        self.action.physical_distance = 0
        self.action.child_need = None
        self.action.parent_need = 0

        # initialise publisher
        self.controller_pub = rospy.Publisher(
            '/parent/controller', Action, queue_size= 0
        )

    def update_messages(self, child_action = None, parent_action = None, child_need = None, parent_need = None):
        # set messages
        self.action.parent = parent_action
        self.action.parent_need = parent_need
        self.action.physical_distance = self.dp
        self.action.emotional_distance = self.de

        # publish
        self.controller_pub.publish(self.action)

    def update_distance(self):
        self.emotional_distance = self.emotional_distance_controller.emotional_distance()
        self.physical_distance = self.physical_distance_controller.physical_distance()

if __name__ == '__main__':
    main = RobotController()
    try:
        main.central_control()
    except rospy.ROSInterruptException:
        pass