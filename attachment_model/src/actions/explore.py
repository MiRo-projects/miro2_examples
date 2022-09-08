#!/usr/bin/env python3

# import core modules
from abc import abstractmethod
import os
import rospy
import numpy as np
import time

# set sys path for getting subscriber modules
import sys
sys.path.append('../')

# import subscription modules
from actions.general_pub import GeneralPub
from subscribers.general_sub import GeneralSub

# import message for odometry
from attachment_model.msg import RobotPub

SPEED = 0.2
MIN_WALL = 0.105

class MiRoExplore():

    def __init__(self):

        # initialise node for publishing and odom message
        self.general_pub = GeneralPub()
        self.general_sub = GeneralSub()
        self.odom_pos = RobotPub()

        # topic base name
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # set robot range
        self.wall_distance = self.general_sub.range
        
        # record start time of MiRo node
        self.start_time = time.time()

    """
        Check whether the robot should turn or go straight
    """
    def check_turn(self):
        time_elasped = time.time() - self.start_time
        # check if robot is in the middle of turning
        if self.explore_robot == False:
            if ((time_elasped % 5) == 0):
                self.explore_robot = True
            print("turning", self.general_sub.range)
            return True
        else:
            # check if the wall is too near
            if self.general_sub.range < MIN_WALL:
                self.start_turn = rospy.get_rostime()
                self.explore_robot = False
                print("wall ahead", self.general_sub.range)
                return True
            else:
                print("going straight")
                return False

    """
        Carry out exploration function
    """
    def explore_action(self):
        time_elasped = time.time() - self.start_time
        if self.check_turn() == False:
            if ((time_elasped % 2) == 0):
                max_angle_turn = np.pi/2 # 90 degrees max angle
                rand_turn = np.random.randint(-np.ceil(max_angle_turn * 10), np.ceil(max_angle_turn * 10))/10
            self.general_pub.set_move_cmd(linear=self.speed, angular=rand_turn)
        else:
            check_dir = np.random.randint(1, 3)
            if ((time_elasped % 5) == 0):
                if check_dir == 1:
                    self.angle_turn = (np.pi/2)/3 * 2
                else:
                    self.angle_turn = -(np.pi/2)/3 * 2
            self.general_pub.set_move_cmd(linear=0, angular=self.angle_turn * 2)
    
    """
        Explore and publish odometry data along the way
    """
    @abstractmethod
    def explore(self):
        pass

class ParentExplore():

    def __init__(self):
        self.odom_publisher = rospy.Publisher(
            self.topic_base_name + '/parent/odom_position', RobotPub, queue_size= 0
        )
        super().__init__()

    def explore(self):
        self.odom_pos.pos_x = self.general_sub.posx
        self.odom_pos.pos_y = self.general_sub.posy
        self.odom_publisher.publish(self.odom_pos)

class ChildExplore():

    def __init__(self):
        self.odom_publisher = rospy.Publisher(
            self.topic_base_name + '/child/odom_position', RobotPub, queue_size= 0
        )
        super().__init__()

    def explore(self):
        self.odom_pos.pos_x = self.general_sub.posx
        self.odom_pos.pos_y = self.general_sub.posy
        self.odom_publisher.publish(self.odom_pos)