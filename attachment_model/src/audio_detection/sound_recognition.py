#!/usr/bin/env python3
#
# The following code has been adapted from client_audio.py provided 
# by CONSEQUENTIAL ROBOTICS for detecting audio

import rospy
import numpy as np
import wave, struct
import matplotlib.pyplot as plt
from audio_detection.process_audio import ProcessAudio
from audio_detection.preprocessing import ProcessMiRo
 
# sample count
SAMPLE_COUNT = 640	#32 ms

class RosCooDetection():

	def listening(self, save = False):
		
		# loop
		detected_sound = np.zeros((0, 1), 'uint16')
		processed_sound = np.empty(0)
		total_filtered_signal = np.empty(0)
		
		while not rospy.is_shutdown():

			self.micbuf = self.mic_data.micbuf
			self.outbuf = self.mic_data.outbuf
			self.startCheck = self.mic_data.startCheck

			if self.startCheck is True and not self.outbuf is None:
				
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

				# set new signal for processing
				self.processing_data.set_signal(outbuf)
				processed_data, accumulation, filtered_signal = self.processing_data.process_miro_detection()
				# collect data from the micbuf for making audio file for re-listening later and also to graph the necessary data
				detected_sound = np.append(detected_sound, outbuf)		# audio
				processed_sound = np.append(processed_sound, processed_data)		# ste
				total_filtered_signal = np.append(total_filtered_signal, filtered_signal)		# filtered sound
				self.mic_data.startCheck = False # to show micbuf data has been processed

				if save == False:
					return accumulation[len(accumulation) - 1]

		if save == True:
			# plot
			plt.figure(figsize=(12, 6))
			plt.subplot(2,2,1)
			plt.plot(detected_sound)
			plt.subplot(2,2,2)
			plt.plot(total_filtered_signal)
			plt.subplot(2,2,3)
			plt.plot(processed_sound)
			plt.subplot(2,2,4)
			plt.plot(accumulation)
			plt.show()

			# save audio file throughout the whole running process
			outfilename = 'tmp/client_audio.wav'	# audio file location
			file = wave.open(outfilename, 'wb')
			file.setframerate(16000)
			file.setsampwidth(2)
			
			print("writing one CENTRE channel to file with sample length " + str(len(detected_sound)))
			file.setnchannels(1)
			for s in detected_sound:
				file.writeframes(struct.pack('<h', s))

			file.close()
	
	"""
		Set starting accumulation to desired one
	"""
	def set_accumulation(self, accumulation):
		self.processing_data.set_accumulation(accumulation)

	def __init__(self):

		# Initialise node and required objects
		#`rospy.init_node("porcupine_sound_recognition")
		self.mic_data = ProcessAudio()

		# state
		self.micbuf = self.mic_data.micbuf
		self.outbuf = self.mic_data.outbuf
		self.startCheck = self.mic_data.startCheck
		self.processing_data = ProcessMiRo()
		# report
		print ("starting to record audio from the right ear of MiRo")