#!/usr/bin/env python

# import core modules
from abc import abstractmethod
import numpy as np

# import subscribers and publishers
from subscribers.action_sub import BothAction, ParentAction, ChildAction
from actions.node_color_change import NodeColorChange
from actions.general_pub import GeneralPub

class GesturesPub(object):
    
    def __init__(self):
        # rgb
        self.general_pub = GeneralPub()
        self.color_control = NodeColorChange()
        self.color_action = self.color_control.convert_color()
        self.color_emotional_distance = self.color_control.convert_color()
        self.color_physical_distance = self.color_control.convert_color()
        self.pos_tail = 0
        self.pos_tail_speed = 0.001

    @abstractmethod
    def set_color(self):
        pass

    def tail_pos_calc(self):
        if self.pos_tail > 0:
            if self.pos_tail_speed > 0:
                if self.pos_tail > 1:
                    self.pos_tail_speed = - 0.0005 * (1 - self.controller.emotional_distance)
        else:
            if self.pos_tail_speed < 0:
                if self.pos_tail < -1:
                    self.pos_tail_speed = 0.0005 * (1 - self.controller.emotional_distance)
        self.pos_tail += self.pos_tail_speed

    def change_color(self):
        print(self.controller.child)
        print(self.controller.emotional_distance)
        self.tail_pos_calc()
        self.wag_pos = np.sin(self.pos_tail)
        self.general_pub.set_cos_joint(wag=self.wag_pos)
        self.set_color()
        self.color_control.set_color_cmd(self.color_emotional_distance, self.color_action, self.color_action, self.color_emotional_distance, self.color_action, self.color_action) 

class BothPGestures(GesturesPub):
    
    def __init__(self):
        self.controller = BothAction()
        super().__init__()
    
    def set_color(self):
        self.color_action = self.color_control.convert_color(blue = (self.controller.parent) * 255, green = (1 - self.controller.parent) * 255)
        self.color_emotional_distance = self.color_control.convert_color(red = ((1 + self.controller.emotional_distance * 0.8) * 135), green= ((1 - self.controller.emotional_distance) * 206), blue = ((1 - self.controller.emotional_distance) *235))  
        self.color_physical_distance = self.color_control.convert_color(red = ((1 + self.controller.physical_distance * 0.8) * 135), green= (self.controller.physical_distance * 206), blue = (self.controller.physical_distance *235))  

class BothCGestures(GesturesPub):

    def __init__(self, type):
        self.controller = BothAction()
        super().__init__()

    def set_color(self):
        self.color_action = self.color_control.convert_color(blue = (self.controller.child) * 255, green = (1 - self.controller.child) * 255)
        self.color_emotional_distance = self.color_control.convert_color(red = ((1 + self.controller.emotional_distance * 0.8) * 135), green= ((1 - self.controller.emotional_distance) * 206), blue = ((1 - self.controller.emotional_distance) *235)) 
        self.color_physical_distance = self.color_control.convert_color(red = ((1 + self.controller.physical_distance * 0.8) * 135), green= (self.controller.physical_distance * 206), blue = (self.controller.physical_distance *235))  

class ParentGestures(GesturesPub):

    def __init__(self):
        self.controller = ParentAction()
        super().__init__()

    def set_color(self):
        self.color_action = self.color_control.convert_color(blue = (self.controller.parent) * 255, green = (1 - self.controller.parent) * 255)
        self.color_emotional_distance = self.color_control.convert_color(red = ((1 + self.controller.emotional_distance * 0.8) * 135), green= ((1 - self.controller.emotional_distance) * 206), blue = ((1 - self.controller.emotional_distance) *235)) 
        self.color_physical_distance = self.color_control.convert_color(red = ((1 + self.controller.physical_distance * 0.8) * 135), green= (self.controller.physical_distance * 206), blue = (self.controller.physical_distance *235))  

class ChildGestures(GesturesPub):

    def __init__(self):
        self.controller = ChildAction()
        super().__init__()

    def set_color(self):
        self.color_action = self.color_control.convert_color(blue = (self.controller.child) * 255, green = (1 - self.controller.child) * 255)
        self.color_emotional_distance = self.color_control.convert_color(red = ((1 + self.controller.emotional_distance * 0.8) * 135), green= ((1 - self.controller.emotional_distance) * 206), blue = ((1 - self.controller.emotional_distance) *235)) 
        self.color_physical_distance = self.color_control.convert_color(red = ((1 + self.controller.physical_distance * 0.8) * 135), green= (self.controller.physical_distance * 206), blue = (self.controller.physical_distance *235))  