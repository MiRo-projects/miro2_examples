#!/usr/bin/env python

# import core modules
import rospy
import numpy as np

# set sys path for getting subscriber modules
import sys
sys.path.append('../')

from subscribers.actions_sub import OdomMiro
from subscribers.action_sub import ActionSub

class ParentData(object):

    def __init__(self):
        rospy.init_node("parent_save_data")
        self.odom_miro = OdomMiro()
        self.action_sub = ActionSub()
        self.pos_data = np.empty([0,2])
        self.need = np.empty(0)
        self.emotional_distance = np.empty(0)
        self.physical_distance = np.empty(0)

    def store_data(self):
        self.pos_data = np.append(self.pos_data, np.array([[self.odom_miro.posx, self.odom_miro.posy]]), axis = 0)
        self.need = np.append(self.need, self.action_sub.child_need)
        self.emotional_distance = np.append(self.emotional_distance, self.action_sub.emotional_distance)
        self.physical_distance = np.append(self.physical_distance, self.action_sub.physical_distance)

    def run(self):
        while not rospy.is_shutdown():
            self.store_data()
        with open('.gitignore/parent_pos_data.npy', 'wb') as f:
            np.save(f, self.pos_data)
        with open('.gitignore/parent_need.npy', 'wb') as f:
            np.save(f, self.need)
        with open('.gitignore/parent_emotional_distance.npy', 'wb') as f:
            np.save(f, self.emotional_distance)
        with open('.gitignore/parent_physical_distance.npy', 'wb') as f:
            np.save(f, self.physical_distance)
        

if __name__ == "__main__":
    data_storage = ParentData()
    data_storage.run()