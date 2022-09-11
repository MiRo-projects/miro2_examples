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
    def __init__(self, args):
        
        self.attend = Attend()
        self.orient = Orient()
        rospy.init_node("sign_stimuli", anonymous=True)
        print("Hello world")

    def loop(self):
        while not rospy.core.is_shutdown():
            
            target, cv = self.attend.activate()
            self.orient.activate( target, cv )
            time.sleep(0.02)
            

if __name__ == "__main__":
    
    main = Client(sys.argv[1:])
    main.loop()