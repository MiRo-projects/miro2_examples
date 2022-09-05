#!/usr/bin/env python3
#
# The following code has been adapted from client_audio.py provided 
# by CONSEQUENTIAL ROBOTICS for detecting audio

import rospy
import numpy as np
import wave, struct
import matplotlib.pyplot as plt
from process_audio import ProcessAudio

# sample count
SAMPLE_COUNT = 640	#32 ms

class ros_coo_detection():

	def loop(self):
		
		# loop
		detected_sound = np.zeros((0, 1), 'uint16')

		# how long to record in seconds
		length_rec = 5
		total_frames = length_rec * 16000

		# init for zcr
		num_frames = int(total_frames/(SAMPLE_COUNT/1.25)) + 1
		zero_crossing_rate = np.zeros(num_frames)
		index_zcr = 0

		while len(detected_sound) < total_frames:

			self.micbuf = self.mic_data.micbuf
			self.outbuf = self.mic_data.outbuf
			self.startCheck = self.mic_data.startCheck

			if self.startCheck is True:
				
				# get audio from left ear of Miro
				detect_sound = np.reshape(self.outbuf[:, [1]], (-1))

				# downsample for playback. sample rate is set to 16000.
				outbuf = np.zeros((int(SAMPLE_COUNT / 1.25), 0), 'uint16')
				i = np.arange(0, SAMPLE_COUNT, 1.25)
				j = np.arange(0, SAMPLE_COUNT)
				x = np.interp(i, j, detect_sound[:])
				outbuf = np.concatenate((outbuf, x[:, np.newaxis]), axis=1)
				outbuf = outbuf.astype(int)
				outbuf = np.reshape(outbuf[:, [0]], (-1))

				# ZCR
				for i in range(1, len(outbuf)):
					zero_crossing_rate[index_zcr] += 0.5 * abs(np.sign(outbuf[i]) - np.sign(outbuf[i - 1]))
				# collect data from the micbuf for making audio file for re-listening later
				detected_sound = np.append(detected_sound, outbuf)
				self.mic_data.startCheck = False # to show micbuf data has been processed
				index_zcr += 1
		t = np.arange(0,5.024,1/16000)
		plt.figure(figsize=(12, 6))
		plt.subplot(2,1,1)
		plt.plot(detected_sound)
		plt.subplot(2,1,2)
		plt.plot(zero_crossing_rate)
		plt.show()

		# save audio file throughout the whole running process
		outfilename = 'tmp/client_audio.wav'	# audio file location
		file = wave.open(outfilename, 'wb')
		file.setframerate(16000)
		file.setsampwidth(2)
		
		print("writing one CENTRE channel to file with sample length " + str(len(detected_sound)))
		file.setnchannels(1)
		for s in detected_sound:
			print(type(s))
			file.writeframes(struct.pack('<h', s))

		file.close()
	
	def __init__(self):

		# Initialise node and required objects
		rospy.init_node("porcupine_sound_recognition")
		self.mic_data = ProcessAudio()

		# state
		self.micbuf = self.mic_data.micbuf
		self.outbuf = self.mic_data.outbuf
		self.startCheck = self.mic_data.startCheck

		# report
		print ("starting to record audio from the right ear of MiRo")

if __name__ == "__main__":
	main = ros_coo_detection()
	main.loop()
