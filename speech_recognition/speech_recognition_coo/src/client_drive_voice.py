#!/usr/bin/python3
#
#	@section COPYRIGHT
#	Copyright (C) 2021 Consequential Robotics Ltd
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

# create node
import rospy
#rospy.init_node("client_drive_voice", anonymous=True)

################################################################

import os
import sys
import time
import numpy as np

import miro2 as miro

################################################################

class controller:

	def callback_package(self, msg):

		# tick @ 50Hz
		self.t_now += 0.02

		# report vbat
		vbat = np.round(np.array(msg.battery.voltage) * 10.0) / 10.0
		if not vbat == self.vbat:
			self.vbat = vbat

	def loop(self):

		# update voice node
		self.pub_animal_state.publish(self.msg)

		# state
		time.sleep(0.1)

	def __init__(self, args):

		# state
		self.t_now = 0
		self.vbat = 0

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish animal state to drive voice
		topic = topic_base_name + "/core/animal/state"
		print ("publish", topic)
		self.pub_animal_state = rospy.Publisher(topic, miro.msg.animal_state,
					queue_size=0)

		# state
		self.msg = miro.msg.animal_state()

		# set emotion
		self.msg.emotion.valence = 1.0 # 0.0 = sad, 1.0 = happy
		self.msg.emotion.arousal = 1.0 # 0.0 = low, 1.0 = high

		# set high ambient sound level to maximize volume
		# (see animal_state.msg for more details)
		self.msg.sound_level = 0.1

		# wakefulness also used as audio gain
		self.msg.sleep.wakefulness = 1.0

		# enable voice
		self.msg.flags = miro.constants.ANIMAL_EXPRESS_THROUGH_VOICE

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()
