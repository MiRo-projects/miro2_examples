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



################################################################


################################################################

class Client:

    def callback_cam(self, ros_image):

        # silently (ish) handle corrupted JPEG frames
        try:

            # convert compressed ROS image to raw CV image
            self.image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

        except CvBridgeError as e:

            # swallow error, silently
            # print(e)
            pass


    def detect_human(self, im):

        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        # resizing for faster detection
        frame = cv2.resize(im, (640, 480))
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        boxes, weights = hog.detectMultiScale(frame, winStride=(16, 16))
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        for (xA, yA, xB, yB) in boxes:
            # display the detected boxes in the colour picture
            cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)

        if len(boxes) == 0:
            return

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return


    def loop(self):
           # loop
        while not rospy.core.is_shutdown():
            if self.image is not None:
                self.detect_human(self.image)
                self.image = None
            time.sleep(0.02)


    def __init__(self, args):

        self.image = None

        rospy.init_node("human_detection_test", anonymous=True)
        # ROS -> OpenCV converter
        self.image_converter = CvBridge()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
                                         CompressedImage, self.callback_cam, queue_size=1, tcp_nodelay=True)
        # self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
        #                                  CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)


if __name__ == "__main__":

    main = Client(sys.argv[1:])
    main.loop()



