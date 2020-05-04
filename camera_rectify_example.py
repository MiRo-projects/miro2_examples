#!/usr/bin/python
# Example of camera correction for MiRo

import sys
sys.path.append('common')
import rospy
import numpy as np
import time

import os
import camera_configs

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import miro2 as miro

################################################################

def error(msg):
    print(msg)
    sys.exit(0)

################################################################

class controller:
    
    def processImage( self, ros_image, index ):
        
        try:
            self.image_converter = CvBridge()
            # convert compressed ROS image to raw CV image
            self.images[index] = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
        except:
            print "An error occurred processing the image"

    def callback_caml(self, ros_image):
        self.processImage( ros_image, 0 )

    def callback_camr(self, ros_image):
        self.processImage( ros_image, 1 )
        

    def rectifyImages( self, images ): # [left_image, right_image]
        img1_rectified = cv2.remap(images[0], \
                         camera_configs.left_map1, \
                         camera_configs.left_map2, \
                         cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(images[1], \
                                   camera_configs.right_map1, \
                                   camera_configs.right_map2, \
                                   cv2.INTER_LINEAR)
        cv2.imshow( 'imageL:', img1_rectified)
        cv2.imshow( 'imageR:', img2_rectified)
        k = cv2.waitKey(100) & 0xFF

        if k == ord('q'):
            self.running = False

    def loop(self):

        while not  rospy.core.is_shutdown():
           
            if self.images[0] is not None and self.images[1] is not None:
                self.rectifyImages( self.images )
                self.images = [None, None]

            time.sleep(0.01)
                    


    def __init__(self, args):
        rospy.init_node("client", anonymous=True)
        
        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # sensors/caml, Frames from the left eye camera (sample rate is variable, see control/command).
        topicCamLeft = topic_base_name+"/sensors/caml/compressed"
        topicCamRight =topic_base_name+"/sensors/camr/compressed"
        
        self.sub_caml = rospy.Subscriber(topicCamLeft, CompressedImage, self.callback_caml, queue_size=1,
                                         tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topicCamRight, CompressedImage, self.callback_camr, queue_size=1,
                                          tcp_nodelay=True)

        self.images = [None, None]
        # wait for connect
        print "wait for connect..."
        time.sleep(1)


if __name__ == "__main__":
    main = controller(sys.argv[1:])

    main.loop()




