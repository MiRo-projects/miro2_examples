#!/usr/bin/env python3
from abc import abstractmethod
import numpy as np
import miro2 as miro
import os
import rospy
import time

from node_joints import NodeJoints
from control_odom import MoveMiro
from node_kinematic_joint import MoveKinematic

class MiroAction:

    """
    The super class contains the basic subscribers and publishers
    required
    Contains an abstract method for all MiRo
    """

    def __init__(self):
        self.robot_joint = NodeJoints()
        self.robot_movement = MoveMiro()
        self.robot_kinematic = MoveKinematic()
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
        self.topic = self.topic_base_name + "/core/animal/state"
        self.pub_animal_state = rospy.Publisher(self.topic, miro.msg.animal_state,
        queue_size=0)
        self.eye_pos = np.sin(time.time() * 3)
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
        joint_pos = np.sin(time.time())
        self.robot_joint.set_joint_cmd(wag = 0, droop = 0, eyel = self.eye_pos, eyer = self.eye_pos, earl = joint_pos, earr = joint_pos)
        self.robot_joint.pub_joint()
        wheel_speed = [2,-2]
        (dr, dtheta) = miro.lib.wheel_speed2cmd_vel(wheel_speed)
        self.robot_movement.set_move_cmd(linear = dr, angular=dtheta)
        self.robot_movement.vel_publish()
        self.robot_kinematic.set_move_kinematic(0)
        self.robot_kinematic.kinematic_publish()

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
        if self.eye_pos < 0.3:
            self.eye_pos = 0.3
        self.robot_joint.set_joint_cmd(wag = 0, droop = 0, eyel = self.eye_pos, eyer = self.eye_pos, earl = -5, earr = -5)
        self.robot_joint.pub_joint()
        self.robot_kinematic.set_move_kinematic(1)
        self.robot_kinematic.kinematic_publish()

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
        joint_pos = np.sin(time.time() * 5)
        self.robot_joint.set_joint_cmd(wag = joint_pos, droop = 0, eyel = self.eye_pos, eyer = self.eye_pos, earl = joint_pos, earr = joint_pos)
        self.robot_joint.pub_joint()
        self.robot_kinematic.set_move_kinematic(0)
        self.robot_kinematic.kinematic_publish()

if __name__ == "__main__":
    while not rospy.core.is_shutdown():
        main = GoodBoy()
        main.execute()