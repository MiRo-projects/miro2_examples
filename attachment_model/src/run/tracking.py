#!/usr/bin/env python3

# capture simultaneously
import cv2
import numpy as np
import math
import rospy
import matplotlib.pyplot as plt

# set sys path for getting subscriber modules
import sys
sys.path.append('../')

from subscribers.child_sound_detection import ChildSoundDetection

class VideoTracking(object):

    """
        Record sound from camera and also save data of sound detection recording
        file_name = file name for existing file or 0 for streaming
    """
    def __init__(self, file_name = 0):
        rospy.init_node("external_data_tracker")

        # for recording camera data
        self.file_name = file_name
        self.capture = cv2.VideoCapture(0)
        self.distance_record = np.empty(0)
        self.pos_record = np.empty([0,2])
        self.record = True

        # sound detection
        self.sound_detection = ChildSoundDetection()
        self.sound_detected = self.sound_detection.care
        self.sound_record = np.empty(0)

    def show_video(self):
        physical_distance = 0
        pixel_x = 0
        pixel_y = 0
        while self.capture.isOpened() and self.record is True and not rospy.is_shutdown():
            self.sound_detected = self.sound_detection.care     # update sound data
            try:
                # read
                ret, frame = self.capture.read()
                # filter the center
                frame[180:320,220:410] = 0
                # grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # binarize
                th, im_th = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
                tracker_point = np.where(im_th > 150)
                # record max distance possibly calculated
                max_distance = math.sqrt((0 - (len(frame[0])/2))**2 + (0 - (len(frame)/2))**2)
                # calculate physical distance and also to filter out a picture of miro estimate location
                if (len(tracker_point[0]) > 0) and (len(tracker_point[1]) > 0):
                    pixel_x = int(np.floor(np.average(tracker_point[0])))
                    pixel_y = int(np.floor(np.average(tracker_point[1])))
                    # get pd
                    physical_distance = math.sqrt(((len(frame[0])/2) - pixel_y)**2 + ((len(frame)/2) - pixel_x)**2)
                    # plot a circle over the point
                    r = 5
                    max_y = pixel_y + r
                    if max_y >= 480:
                        max_y = 480
                    min_y = pixel_y - r
                    if min_y < 0:
                        min_y = 0
                    y_range = np.arange(min_y, max_y)
                    for y in y_range:
                        min_x = int(pixel_x - math.sqrt((r**2) - ((y - pixel_y) ** 2)))
                        if min_x < 0:
                            min_x = 0
                        max_x = int(pixel_x + math.sqrt((r**2) - ((y - pixel_y) ** 2)))
                        if max_x >= 640:
                            max_x = 640
                        im_th[min_x:max_x, y] = 255
                else:
                    physical_distance += 1
                    if physical_distance >= max_distance:
                        physical_distance = max_distance
                # record distance
                self.distance_record = np.append(self.distance_record, physical_distance)
                # record position
                position = np.array([[pixel_x, pixel_y]])
                self.pos_record = np.append(self.pos_record, position, axis = 0)
                self.sound_record = np.append(self.sound_record, self.sound_detected)
                cv2.imshow('frame', frame)
                c = cv2.waitKey(1)
                if c == 27:
                    break
            except KeyboardInterrupt:
                break
            except:
                continue
        self.capture.release()
        cv2.destroyAllWindows()
        plt.figure()
        plt.plot(self.distance_record)
        plt.figure()
        plt.plot(self.pos_record[:,0], self.pos_record[:,1])
        plt.figure()
        plt.plot(self.sound_record)
        plt.show()

if __name__ == "__main__":
    capture = VideoTracking()
    capture.show_video()