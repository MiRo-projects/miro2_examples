#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Float32MultiArray

class NodeJoints(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.joint_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

    # Making joints move
    def set_joint_cmd(self, droop=0.0, wag=1.0, eyel=0.0, eyer=0.0, earl=1.0, earr=1.0):
        self.joint_move = Float32MultiArray()
        self.joint_detail = [droop, wag, eyel, eyer, earl, earr]
        self.joint_move.data = self.joint_detail

    def pub_joint(self):
        self.joint_pub.publish(self.joint_move)
