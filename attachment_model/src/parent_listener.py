#!/usr/bin/env python3
#
# Run listener node for parent

from audio_detection.actions import ParentListening

class Listener():

    def __init__(self):
        self.listener = ParentListening()

if __name__ == "__main__":
    listener = Listener()
    listener.listener.action()