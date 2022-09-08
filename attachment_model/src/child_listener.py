#!/usr/bin/env python3
#
# Run listener for child

from audio_detection.actions import ChildListening

class Listener():

    def __init__(self):
        self.listener = ChildListening()

if __name__ == "__main__":
    listener = Listener()
    listener.listener.action()