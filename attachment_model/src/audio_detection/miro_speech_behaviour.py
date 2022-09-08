#!/usr/bin/env python3

"""
Supposedly the MiRo's mass is set as 2 and thus this will decide 
the frame size and the frequency range of the bandpass filter
"""

import numpy as np

class MiRoSpeech():

    """
        M for mass of animal
        m for mouth opening (0 = open, 1 = closed)
        c for speed of sound (in cm/s)
        n for the harmonics
    """
    def __init__(self, M, m = 0, c = 34300, n = 1):
        self.M = M
        self.m = m
        self.c = c
        self.n = n
        vocaltract = self.VocalTract(self.M)
        self.L = vocaltract.decide_vocal_tract_length()

    """
        Rate at which the MiRo breathes. This can be used as a reference of frame for the MiRo.
    """
    def breathing_rate(self):
        B = 0.84 * (self.M ** -0.26)
        return B
    
    """
        This will be used to decide the frequency range for the multi-bandpass filter
    """
    def resonant_frequency(self):
        R = ((2 * self.n) - (self.m + 1)) * (self.c/(4 * self.L))
        return R
    
    """
        Set the frequency range to obtain in a format of array that will be used
    """
    def getFrequencyList(self, noFrequency = 1, difference = 100):
        frequency_list = np.empty([0, 2])
        for i in range(noFrequency):
            self.n = i + 1
            frequency_range_add = np.array([[self.resonant_frequency() - difference, self.resonant_frequency() + difference]])
            frequency_list = np.append(frequency_list, frequency_range_add, axis= 0)
        return frequency_list

    """
        Description of the vocal tract
    """
    class VocalTract():

        """
            M for mass of animal
        """
        def __init__(self, M):
            self.M = M
        
        """
            Length of vocal tract to decide on the frequency range
        """
        def decide_vocal_tract_length(self):
            L = 3.15 + (11.53 * np.log(self.M))
            return L