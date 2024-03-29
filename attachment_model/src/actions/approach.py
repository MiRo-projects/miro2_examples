#!/usr/bin/env python3

# import core modules
from abc import abstractmethod
import rospy
import os
import time

# import subscribers and modules
from subscribers.camera_sub import MiRoCamera
from subscribers.general_sub import GeneralSub
from actions.apriltag_perception import AprilTagPerception
from actions.general_pub import GeneralPub

# import messages
from attachment_model.msg import RobotPub

SPEED = 0.5
MIN_RANGE = 0.35

class MiRoApproach(object):
    def __init__(self):
        # set variables
        self.tag_size = 1
        self.tag_found = False              # check for tags
        self.condition_satisfied = False    # check if MiRo is near tag
        
        # import required subscribers and modules
        self.miro_camera = MiRoCamera()
        self.april_tag = AprilTagPerception(size=self.tag_size, family='tag36h11')
        self.general_sub = GeneralSub()
        self.general_pub = GeneralPub()

        # required variables for camera and tag
        self.input_camera = self.miro_camera.input_camera
        self.new_frame = self.miro_camera.new_frame
        self.range = self.general_sub.range
        self.tag = [None, None]
        self.start_time = time.time()
        self.current_time = time.time() - self.start_time
        self.condition_satisfied = False

    """
        Update camera, frame and range
    """
    def update_variables(self):
        self.input_camera = self.miro_camera.input_camera
        self.new_frame = self.miro_camera.new_frame
        self.range = self.general_sub.range
        self.current_time = time.time() - self.start_time

    """
        April tag detection
    """
    def lookForTag(self, set_id = None):
        
        # detect the tags and identify their id
        for index in range(2):
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            self.tag[index] = self.april_tag.detect_tags(image)
        
        # check all tag if None
        if set_id is None:
            if not self.tag[0] or not self.tag[1]:
                if ((self.current_time % 2) < 0.2):
                    self.general_pub.drive(SPEED, -SPEED)
                    self.tag_found = False
            else:
                self.tag_found = True
        else:
            # if tag is not found and the tag id is not valid for left and right eye
            if not (self.tag[0] and ((self.tag[0][0].id in set_id) or (self.tag[1][0].id in set_id))):
                self.general_pub.drive(SPEED, -SPEED)
                self.tag_found = False
            else:
                self.tag_found = True

    """
        Approach april tag on detection
    """
    def approach_func(self, set_id = None):
        self.update_variables()
        self.lookForTag(set_id)
        if not self.condition_satisfied:
            if self.tag_found == True:
                if self.tag[0] and self.tag[1]:
                    self.general_pub.drive(SPEED, SPEED)
                # if in left camera turn left
                elif self.tag[0] and not self.tag[1]:
                    self.general_pub.drive(0, SPEED)
                # if in right camera turn right
                elif not self.tag[0] and  self.tag[1]:
                    self.general_pub.drive(SPEED, 0)
                if self.range < MIN_RANGE:
                    self.condition_satisfied = True
        else:
            print("DONE")
    """
        Approach tag with odom data being published    
    """
    @abstractmethod
    def approach(self):
        pass
    
class ParentApproach(MiRoApproach):

    def __init__(self):

        # topic base name
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publisher
        self.odom_publisher = rospy.Publisher(
            '/parent/odom_position', RobotPub, queue_size= 0
        )

        # odom message
        self.odom_pos = RobotPub()

        super().__init__()

    def approach(self):
        self.approach_func()
        self.odom_pos.pos_x = self.general_sub.posx
        self.odom_pos.pos_y = self.general_sub.posy
        self.odom_publisher.publish(self.odom_pos)

class ChildApproach(MiRoApproach):

    def __init__(self):

        # topic base name
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publisher
        self.odom_publisher = rospy.Publisher(
            '/child/odom_position', RobotPub, queue_size= 0
        )

        # odom message
        self.odom_pos = RobotPub()

        super().__init__()

    def approach(self):
        self.approach_func()
        self.odom_pos.pos_x = self.general_sub.posx
        self.odom_pos.pos_y = self.general_sub.posy
        self.odom_publisher.publish(self.odom_pos)