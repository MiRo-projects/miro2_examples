#!/usr/bin/env python3
from abc import abstractmethod
import numpy as np
import miro2 as miro
import os
import rospy
import time

from node_wag_tail import NodeWagTail
from control_odom import MoveMiro

class MiroAction:

    """
    The super class contains the basic subscribers and publishers
    required
    Contains an abstract method for all MiRo
    """

    def __init__(self):
        self.robot_wag = NodeWagTail()
        self.robot_movement = MoveMiro()
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
        self.topic = self.topic_base_name + "/core/animal/state"
        self.pub_animal_state = rospy.Publisher(self.topic, miro.msg.animal_state,
        queue_size=0)

        #sound
        self.msg = miro.msg.animal_state()

    @abstractmethod
    def execute(self):
        pass


class HeyMiro(MiroAction):

    """
    The HeyMiro subclass is supposed to spin the MiRo only
    """

    def __init__(self):
        super().__init__()

    def execute(self):
        angular_velo = 2 * np.pi/3
        self.robot_movement.set_move_cmd(angular=angular_velo)
        self.robot_movement.vel_publish()


class NoMiro(MiroAction):

    """
    The NoMiro subclass makes the Miro stop what they are doing,
    droop its ears and coo sadly.
    """

    def __init__(self):
        super().__init__()
        # set coo to be sad, low arousal and soft
        self.msg.emotion.valence = 0
        self.msg.emotion.arousal = 0.6
        self.msg.sound_level = 0.1
        self.msg.sleep.wakefulness = 1.0
        self.msg.flags = miro.constants.ANIMAL_EXPRESS_THROUGH_VOICE

    def execute(self):
        # stop robot movement
        self.robot_movement.stop()
        # start making sound
        self.pub_animal_state.publish(self.msg)

class GoodBoy(MiroAction):

    """
    The HeyMiro subclass makes the Miro stop what they are doing,
    start wagging tail and coo happily.
    """

    def __init__(self):
        super().__init__()
        # set coo to be happy, high arousal and loud
        self.msg.emotion.valence = 1
        self.msg.emotion.arousal = 1
        self.msg.sound_level = 0.1
        self.msg.sleep.wakefulness = 1.0
        self.msg.flags = miro.constants.ANIMAL_EXPRESS_THROUGH_VOICE

    def execute(self):
        # stop robot movement
        self.robot_movement.stop()
        #start making sound
        self.pub_animal_state.publish(self.msg)
        # start wagging tail
        tail_pos = np.sin(time.time() * 1.8)
        self.robot_wag.set_wag_cmd(tail_pos)
        self.robot_wag.pub_wag()

if __name__ == "__main__":
    while not rospy.core.is_shutdown():
        main = HeyMiro()
        main.execute()