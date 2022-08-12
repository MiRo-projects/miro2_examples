#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from sensor_msgs.msg import JointState  # ROS cmd_vel (velocity control) message

class MoveKinematic(object):

    def __init__(self):
        # Individual robot name acts as ROS topic prefix
        self.joint_pos = JointState()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.kinematic_pub = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )

    def set_move_kinematic(self, neck = 0):
        self.joint_pos.position = [0, neck, 0, 0] 

    def kinematic_publish(self):
        self.kinematic_pub.publish(self.joint_pos)