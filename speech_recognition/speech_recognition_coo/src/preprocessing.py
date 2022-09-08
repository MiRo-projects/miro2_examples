#!/usr/bin/env python3

"""
    The followning code will be made to filter the MiRo's voice will be filtered out from the background noise.
    This will then be used as preprocessing for the training data. The file is also used for 
    getting an image of the signal, its spectral domain plot and its spectrogram plot.
"""

import wave, struct
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, freqz
from abc import abstractmethod

# import from local file
from process_zcr_energy import ProcessEnergyZCR
from miro_speech_behaviour import MiRoSpeech
from accumulator import Accumulator

"""
    Parent class to filter and process audio
"""
class Preprocessing(object):

    """
        CONSTRUCTOR:
        Parent Class for filtering the audio
            filter_range: an array of frequency range for filtering
            file_location: the file location of audio to be processed
            file_name: file name to be saved as
            signal: if None it will use signal from the file location
                    if signal has been passed on, it will use that as a stream audio
            fs: sampling frequency (default: 16k Hz)
            frame_size: frame size of audio used
    """
    def __init__(self, filter_range, file_location = None, file_name = "processed_audio", signal = None, fs = 16000, frame_size = 512):
        
        self.file_name = file_name

        # for sampling frequency (default = 16k)
        self.fs = fs
        
        # filter range must be a nested array with the array inside having 2 int only of range greater than 0
        self.filter_range = filter_range

        # frame size
        self.frame_size = frame_size
        self.frame_seconds = self.frame_size/self.fs

        # decide whether to stream signal or get from existing file
        if not signal is None:
            self.signal = signal
        elif signal is None and not file_location is None:
            with wave.open(file_location, 'r') as wav_file:
                self.signal = wav_file.readframes(-1)
                self.signal = np.fromstring(self.signal, 'Int16')
        else:
            self.signal = None
            
        self.activation_func = Accumulator(k=1)
        
    """
        Set for new signal
    """
    def set_signal(self, signal):
        self.signal = signal
    
    """
        Spectral Domain Plot
    """
    def get_spectral_domain(self, limit = None):
        LDFT = 2**14 # DFT length
        f1=np.linspace(0,self.fs/2,int(LDFT/2))    # the positive frequencies(up to fs/2)
        f2=np.linspace(-self.fs/2,0,int(LDFT/2))   # the negative frequencies
        f = np.concatenate([f1, f2])
        X = np.fft.fft(self.signal,LDFT)

        plt.figure(figsize=(24,8))
        plt.plot(f, np.abs(X))
        plt.title('signal in spectral domain')
        plt.xlabel('frequency $f$ in Hz')
        plt.ylabel('$|x(f)|$')
        plt.ylim([0,50000])
        if not limit is None:
            plt.xlim([-limit, limit])
        plt.show()
    
    """
        Spectrogram Plot
    """
    def get_spectrogram(self):
        plt.specgram(self.signal, Fs=self.fs)
        plt.xlabel('time $t$')
        plt.ylabel('frequency $f$')
        plt.grid(None)
        plt.colorbar(label='dB')
        plt.clim(-50,0)
        plt.tight_layout()
        plt.show()
    
    """
        Plot frequency response for multiple order and range
    """
    def plot(self):
        plt.figure(1)
        plt.clf()
        for i in self.filter_range:
            for order in [3, 6, 9]:
                b, a = self.butter_bandpass(i[0], i[1], order=order)
                w, h = freqz(b, a, worN=2000)
                plt.plot((self.fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % order)

            plt.plot([0, 0.5 * self.fs], [np.sqrt(0.5), np.sqrt(0.5)],
                    '--', label='sqrt(0.5)')
            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Gain')
            plt.grid(True)
            plt.legend(loc='best')
            plt.show()

    """
        Butterworth filter
    """
    def butter_bandpass(self, lowcut, highcut, order=5, data = None):
        nyq = 0.5 * self.fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        if data is None:
            return b, a
        else:
            y = lfilter(b, a, data)
            return y
    
    """
        Multi-band pass filter using butterworth
            save: decide whether to save the filtered signal or not
    """
    def get_filtered_signal(self, save = True):

        total_filtered_signal = np.zeros(len(self.signal), dtype=np.int64)      # filtered signal after working on a multi-band pass
        for i in self.filter_range:
            filtered_signal = np.array(self.butter_bandpass(i[0], i[1], order=6, data = self.signal), dtype=np.int64)
            total_filtered_signal = np.add(filtered_signal, total_filtered_signal)

        # save signal if required
        if save == True:
            # save audio file for the filtered_signal
            outfilename = 'tmp/' + self.file_name + '.wav'	# audio file location
            file = wave.open(outfilename, 'wb')
            file.setframerate(self.fs)
            file.setsampwidth(2)
            
            print("writing the filtered signal to file " + str(outfilename)) 
            file.setnchannels(1)
            for s in total_filtered_signal:
                file.writeframes(struct.pack('<h', s))
            file.close()
            print("Completed saving")
        return total_filtered_signal
    
    """
        Processing Zero Crossing Rate or Short Term Energy
    """
    def process_filtered_signal(self, method = "zcr", filter = True, save = False):

        signal = self.signal

        # filter signal if required
        if filter == True:
            signal = self.get_filtered_signal(save)

        # total time of the signal
        check_frame_seconds = len(signal)/self.fs

        # check whether the signal is eligible in one frame or not
        if check_frame_seconds == self.frame_seconds:
            process_signal = ProcessEnergyZCR(signal, self.fs)
            if method == "zcr":
                processed_signal = np.array([process_signal.calc_ZCR()])
            elif method == "ste":
                processed_signal = np.array([process_signal.calc_STE()])
            else:
                raise Exception("Choose either zcr or ste for method")

        # if frame length is bigger than allowed it will assume a full audio is playing
        elif check_frame_seconds > self.frame_seconds:
            nFrames = int(len(signal)/(self.fs * self.frame_seconds))
            processed_signal = np.zeros(nFrames)
            # process for each frame which is for 32ms
            for frame in range(nFrames):
                startIdx = int(frame * (self.fs * self.frame_seconds))
                stopIdx = int(startIdx + (self.fs * self.frame_seconds))
                process_signal = ProcessEnergyZCR(signal[startIdx:stopIdx], self.fs)
                if method == "zcr":
                    processed_signal[frame] = process_signal.calc_ZCR()
                elif method == "ste":
                    processed_signal[frame] = process_signal.calc_STE()
                else:
                    raise Exception("Choose either zcr or ste for method")
                    
        # frame length is too short to process
        else:
            raise Exception("frame length must be at least for 0.32ms")
        return signal, processed_signal

    """
        Pass through an accumulator at specific frequency range for MiRo detection.
        Values and methods can be different for other speeches.
    """
    @abstractmethod
    def process_miro_detection(self):
        pass

"""
    Subclass made for detecting miro only
"""
class ProcessMiRo(Preprocessing):

    """
        CONSTRUCTOR
            filter_range: uses value obtained from the miro speech class
            mass: mass of the miro (or any mammal that may be used for reference)
            passing to values for super class
    """
    def __init__(self, file_location = None, file_name = "processed_audio", signal = None, fs = 16000, frame_size = 512, mass = 2):
        miroSpeechSettings = MiRoSpeech(mass)
        self.accumulation = np.empty(0)
        filter_range = miroSpeechSettings.getFrequencyList(noFrequency= 3, difference= 100)
        self.store_accumulation = 0
        super().__init__(filter_range, file_location=file_location, file_name= file_name, signal = signal, fs =fs, frame_size=frame_size)

    def set_accumulation(self, accumulation):
        self.accumulation = accumulation
        self.activation_func.set_x(accumulation)

    """
        Calculate the ste and check if another miro has produced sound through the use of an accumulator
    """
    def process_miro_detection(self, upper_limit = 2500000, filter = True):
        # filter the sound
        filtered_signal, processed_signal = self.process_filtered_signal(method = "ste", filter=filter)
        mod_data = processed_signal - upper_limit
        processed_data = np.heaviside(mod_data, 0)
        for data in processed_data:
            self.accumulation = np.append(self.accumulation, self.activation_func.check_sound(data))
        """
        self.accumulation[-1] > t1:
            setAllinouttoZero()
            generateCou(2s)
            Listen again

        self.accumulation[-1] > t2:
            print("Happy")
        """
        return processed_data, self.accumulation, processed_signal