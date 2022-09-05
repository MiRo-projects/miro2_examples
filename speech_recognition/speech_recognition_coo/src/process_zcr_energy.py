#!/usr/bin/env python3

import numpy as np

"""
    The preprocessing will have the whole frame used since
    the streamed signal will decide the frame length
    This module will process the short term energy and zcr for each frame
"""
class ProcessEnergyZCR():

    def __init__(self, signal = None, fs = 16000):
        self.signal = signal
        self.fs = fs
        self.frame_length = fs * 32/1000    # around 512 frames for 16000 sample rate so 32ms
        if len(self.signal) != self.frame_length:
            raise Exception("The frame length should be of size " + str(self.frame_length))
            
    """
        Calculate short term energy for the frame
    """
    def calc_STE(self):
        short_term_energy = np.sum(self.signal[0:len(self.signal)] ** 2)
        return short_term_energy
        
    """
        Calculate zero crossing rate for the frame
    """
    def calc_ZCR(self):
        ZCR = 0
        for k in range(1, len(self.signal)):
            ZCR += 0.5 * abs(np.sign(self.signal[k]) - np.sign(self.signal[k - 1]))
        return ZCR