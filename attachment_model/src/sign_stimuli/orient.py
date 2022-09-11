from telnetlib import NOP
from this import d
import numpy as np
import os
from geometry_msgs.msg import Twist, TwistStamped


from sensor_msgs.msg import JointState
import rospy

class State(object):
    def execute(self, target, vel):
        NOP

class ListenState(State):
    def execute(self, orient):
        if orient.vel > 0:
            return MoveState(  orient.target, orient.vel  )

        return self

class MoveState(State):
    def __init__(self, target, vel ):
        self.target = target
        self.vel = vel
        self.t = 0.0

    def execute(self, orient):
        msg_kin = JointState()

        msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]
        msg_kin.position[1] = np.radians(50.0)
        msg_kin.position[2] = np.radians(-self.t*self.target)
        msg_kin.position[3] = np.radians(-60.0)

        orient.pub_kin.publish(msg_kin)

        self. t += 0.8

        if self.t > 50.0: 
            return RelaxState(self.target, self.vel)

        return self

class RelaxState(State):
    def __init__(self, target, vel):
        self.target = target
        self.vel = vel

    def execute(self, orient):
        msg_kin = JointState()
        kp = np.radians(0.0)

        msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]
        msg_kin.position[1] = np.radians(50.0)
        msg_kin.position[2] = kp
        msg_kin.position[3] = np.radians(-60.0)

        orient.pub_kin.publish(msg_kin)

        if np.linalg.norm(np.asarray(orient.kinpos) - np.asarray(orient.kinpos_old)) == 0:
            return RotateState( self.target, self.vel )

        return self

class RotateState( State ):
    def __init__(self, target, vel):
        self.target = target
        self.vel = vel
        self.t = 0

    def execute( self, orient ):
        
        if self.t < 40:
            msg_wheels = TwistStamped()
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = -self.target
            orient.pub_wheels.publish(msg_wheels)

        self.t += 1

        if self.t > 60:
            return ListenState()
        
        return self

class Orient:
    def __init__(self):

        self.kinpos = None
        self.kinpos_old  = None
        self.state = ListenState()
        self.thres = 3.0
        self.acc_vel = 0
        self.image_w = 640
        self.target = self.image_w
        

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
         # publish
        topic = topic_base_name + "/control/kinematic_joints"
        print ("publish", topic)
        self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)
        # subscribe
        topic = topic_base_name + "/sensors/kinematic_joints"
        print ("subscribe", topic)
        self.sub_package = rospy.Subscriber(topic, JointState, self.callback_kin, queue_size=5, tcp_nodelay=True)
        # publish
        topic = topic_base_name + "/control/cmd_vel"
        print ("publish", topic)
        self.pub_wheels = rospy.Publisher(topic, TwistStamped, queue_size=0)


    def callback_kin(self, data):
        if self.kinpos_old is None: 
            self.kinpos = data.position
    
        
        self.kinpos_old = self.kinpos
        self.kinpos = data.position

    def activate(self, target, vel):
        self.target = target
        self.vel = vel

        self.state = self.state.execute( self )