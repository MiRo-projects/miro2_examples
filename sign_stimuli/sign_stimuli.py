#!/usr/bin/python
#
#	@section COPYRIGHT
#	Copyright (C) 2020 Consequential Robotics Ltd
#
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

import rospy
from sensor_msgs.msg import CompressedImage

import time
import sys
import os
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from orient import *
from attend import *



################################################################


################################################################



class Client:

    def callback_caml(self, ros_image):

       self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):
    
       self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, imno):
    
        # silently (ish) handle corrupted JPEG frames
        try:
            # convert compressed ROS image to raw CV image
            self.image[imno] = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
        except CvBridgeError as e:
            # print(e)
            pass
       
   

    def loop(self):
        

        while not rospy.core.is_shutdown():
            # print(self.moving_hold)
            ip1 = ip2 = None
            v1 = v2 = 0

            if self.image[0] is not None:
                ip1, v1 = self.attend.activate(self.image[0], 0)
            if self.image[1] is not None:
                ip2, v2 = self.attend.activate(self.image[1], 1)            
            
            target = 0.0
            cv = 0.0 

            if ip2 is not None or np.abs(v2) > np.abs(v1):
                target = 0 if ip2 == 0 else 1
                cv = v2
            elif ip1 is not None or np.abs(v1) > np.abs(v2):
                target = -1 if ip1 == 0 else 0
                cv  = v1
            else:
                cv = 0.0

            # print("Movement in: ", cip, ", vel: ", cv, ", active: ", self.orient.active)

            self.orient.activate( target, cv )          
        
            self.image = [None, None]
            time.sleep(0.02)


    def __init__(self, args):
    
        
        self.attend = Attend()
        self.orient = Orient()
        self.image = [None, None]
        self.image_w = 640
        self.image_h = 0
        self.initial_point = np.array([0, 0])
        self.current_vel = 0
        self.t = 0.0
        self.curr_pos = np.radians(0.0)
        self.idx = 0

        rospy.init_node("sign_stimuli", anonymous=True)
        # ROS -> OpenCV converter
        self.image_converter = CvBridge()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

        print("Hello world")



if __name__ == "__main__":
    
    main = Client(sys.argv[1:])
    main.loop()



