#!/usr/bin/env python3

# import messages
from sensor_msgs.msg import CompressedImage

# import modules
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os
import cv2

class MiRoCamera:
    
    def __init__(self):
        # initialisation
        self.input_camera = [None, None]
        self.new_frame = [False, False]
        self.image_convert = CvBridge()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # camera subscribers
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size = 1,
            tcp_nodelay = True
        )

        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size = 1,
            tcp_nodelay = True
        )

    def callback_caml(self, ros_image):
        self.callback_cam(ros_image, 0)
    
    def callback_camr(self, ros_image):
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        try:
            image = self.image_convert.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.input_camera[index] = image
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            self.new_frame[index] = True
        except CvBridgeError as e:
            pass
