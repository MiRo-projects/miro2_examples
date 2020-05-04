#!/usr/bin/python
#This is a MIRO ROS client for Python.
import sys
sys.path.append('../common')
import rospy
from geometry_msgs.msg import TwistStamped
import datetime
import math
import numpy as np
import time

import os
import cal_depth
import camera_configs
import alldistance

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import miro2 as miro
# from detect_primary_user import *

# The miro2 Python module provides constants and tools for working with MiRo from Python.
# with PYTHONPATH=~/mdk/share/python:$PYTHONPATH
import miro2 as miro
import pandas as pd

################################################################

def error(msg):
    print(msg)
    sys.exit(0)

################################################################

class controller:
    def callback_package(self, msg):
        self.sonar = msg.sonar.range
        # print(self.sonar)

    def callback_caml(self, ros_image):
        self.i += 1

        # ignore until active
        if not self.active:
            return

        self.image_converter = CvBridge()
        # convert compressed ROS image to raw CV image
        self.outImage_left= self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
 
    def callback_camr(self, ros_image):

        self.i += 1

        # ignore until active
        if not self.active:
            return

        try:

            self.image_converter = CvBridge()
            # convert compressed ROS image to raw CV image
            self.outImage_right = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
         
            if not self.outImage_right is None or self.outImage_left is None:

                self.threeD=cal_depth.cal_dep(self.outImage_left,self.outImage_right)
        except:
            print "An error occurred processing the image"
        

    def action_forward(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()

            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            # time.sleep(1)
            else:
                self.velocity.twist.linear.x = 0.5
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_l45(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.79
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_r45(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = -0.79
                self.pub_cmd_vel.publish(self.velocity)



    def action_turn_l90(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.79
                self.pub_cmd_vel.publish(self.velocity)


    def action_stepback(self):
        start = datetime.datetime.now()

        while (self.sonar <0.58 or self.sonar >0.65):
            end = datetime.datetime.now()
            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = -0.25
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)


    def action_stop(self):
        start = datetime.datetime.now()

        while (True):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)

    def move(self, action):
        # action = self.action_space[action_i]
        print "action", action

        # give the speed to go 0.5m and stop
        if action == 'forward':
            self.action_forward()
            reward=2

        # turn 45 degree
        elif action == 'left':
            self.action_turn_l45()
            reward = 0.3

        elif action == 'right':
            self.action_turn_r45()
            reward = 0.3

        # turn -90 degree
        elif action == 'back':
            self.action_stepback()

        time.sleep(1)
        if self.sonar<0.5 or self.sonar>0.65:
            reward=-10
            done=True
            self.action_stepback()
        else:
            done=False

        time.sleep(0.3)
        self.average = alldistance.calalldis(self.threeD)
        next_state = self.find_state(self.average)
        return reward,next_state,done



    def find_state(self,average):
        state3=""
        state4=[]
        for i in range(0, 3):
            state2 = str(average[i])
            state3 = state3 + state2
        state4.append(int(state3))
        return state4

    def save_QTable(self,Q_table):
        Q_table.to_csv("qtable17.csv")

    def save_reward(self,datas):
        import xlwt
        import codecs
        import csv
        f=xlwt.Workbook()
        sheet1=f.add_sheet(u"sheet1",cell_overwrite_ok=True)
        i=0
        j=0
        for i in range(len(datas)):

                sheet1.write(i,j,datas[i])
                i=i+1
                f.save("reward.xls")

    def read_QTable(self):
        Q_table = pd.read_csv("qtable8.csv",index_col=0)
        return Q_table

    def plot_reward(self):
        import matplotlib.pyplot as plt
        plt.figure(2)
        plt.plot(np.arange(len(self.tot_reward)),self.tot_reward)
        plt.ylabel("cost")
        plt.xlabel("training steps")
        plt.savefig("reward.png")

    def loop(self):
        # loop
        self.learningtable.q_table=self.read_QTable()
        time.sleep(1)
        # # statetest=[295330350]
        # # if str(statetest) not in self.learningtable.q_table.index:
        # #     print("not exist")
        # # else:
        # #     print("aready exist")
        #
        while self.active and not rospy.core.is_shutdown():
            # print(self.threeD)
            # print(self.average)
            # print(self.sonar)
            #cv2.imshow('imager', self.outImage_right)
            #cv2.imshow('imagel', self.outImage_left)
            allreward=4919
            j=0
            for i in range(self.MAX_EPISODES):

                if rospy.core.is_shutdown():
                    break

                self.average = alldistance.calalldis(self.threeD)
                state = self.find_state(self.average)
                # print("i", i)
                done = False
                # tot_reward = 0
                step = 0

                while not done:
                    print "Choosing action"

                    action = self.learningtable.choose_action(str(state))
                    # if self.sonar < 0.05:
                    #     action = 'STEP_BACK'
                    # # Get next state and reward
                    time.sleep(0.3)
                    reward, next_state, done = self.move(action)

                    if step == 1000:
                        done = True
                        state2 = 'terminal'
                    # if (step >50) and (step % 50 ==0):
                    #     self.plot_reward()
                    #     self.learningtable.plot_cost()
                    #     self.save_reward(self.tot_reward)

                    # Allow for terminal states
                    #self.learningtable.learn(str(state), action, reward, str(next_state))

                    state = next_state
                    allreward =allreward+ reward
                    self.tot_reward.append(allreward)
                    # self.save_reward(self.tot_reward)
                    # print("steps", step)
                    # print("reward", reward)
                    # print("new state", state)
                    # print(self.sonar)
                    time.sleep(0.01)
                    step += 1
                    j+=1
                    if (j >50) and (j % 500 ==0):
                        # self.save_reward(self.tot_reward)
                        self.plot_reward()
                        # self.learningtable.plot_cost()

                    # self.plot_reward()
                    # self.learningtable.plot_cost()
                    # self.save_QTable(self.learningtable.q_table)
                self.save_QTable(self.learningtable.q_table)
            break
        # cv2.destroyAllWindows()


    def __init__(self, args):
        rospy.init_node("client", anonymous=True)
        
        # sim
        self.t = time.time()
        self.i = 0
        self.j = 0

        # state
        self.t_now = 0.0
        self.active = False

        # inputs
        self.package = None
        self.detected_faces = None
        self.roi_color = None

        self.outImage_left =None
        self.outImage_right=None
        self.average=None
        self.sonar=None
        self.threeD=None
        self.state=""
        self.state1= ""

	# init aws rekognition
	# self.rekognition = boto3.client('rekognition', region_name='us-east-2')
        self.collectionId = 'primary_user'
        # create a collection
        # rekognition.create_collection(CollectionId=collectionId)
        # path of the training pics library of the primary user

        self.EPSILON = 0.9  # greedy police
        self.ALPHA = 0.1  # learning rate
        self.GAMMA = 0.9  # discount factor
        self.MAX_EPISODES = 9000  # maximum episodes
        self.learningtable = BulidQTable()
        # self.cost=0
        self.tot_reward=[]

        # handle args
        for arg in args:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f + 1:]
            if key == "pass":
                pass
            else:
                error("argument not recognised \"" + arg + "\"")

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.velocity = TwistStamped()


        # publish
        # topic = topic_base_name + "/control/cmd_vel"
       # print("publish", topic)
       # self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

        # subscribe
        # sensors/caml, Frames from the left eye camera (sample rate is variable, see control/command).
        topicCamLeft = topic_base_name+"/sensors/caml/compressed"
        topicCamRight =topic_base_name+"/sensors/camr/compressed"
        topicpackage = topic_base_name+"/sensors/package"
        topicvel=topic_base_name+"/control/cmd_vel"
        print("subscribe", topicCamLeft)
        self.sub_caml = rospy.Subscriber(topicCamLeft, CompressedImage, self.callback_caml, queue_size=1,
                                         tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topicCamRight, CompressedImage, self.callback_camr, queue_size=1,
                                          tcp_nodelay=True)
        self.sub_package = rospy.Subscriber(topicpackage, miro.msg.sensors_package,
                                            self.callback_package)
        self.pub_cmd_vel = rospy.Publisher(topicvel, TwistStamped, queue_size=0)


        # wait for connect
        print "wait for connect..."
        time.sleep(1)

        # set to active
        self.active = True


class BulidQTable:
    def __init__(self,learning_rate=0.2, discount=0.9, e_greedy=0.9):
        self.N_STATES = 540  # the length of the 1 dimensional world
        self.actions = ['left', 'right', 'forward']  # available actions
        self.EPSILON = 0.9  # greedy police
        self.ALPHA = 0.1  # learning rate
        self.GAMMA = 0.9  # discount factor
        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)
        self.cost=0
        self.costall = []
        self.learn_step=0

        # self.q_table = self.read_QTable()
        # print('Second: {}'.format(self.q_table))

    def check_state_exist(self, state):
        # print( 'columns: {}'.format(self.q_table.columns))
        if state not in self.q_table.index:
            # print( self.q_table.columns )

            # append new state to q table
            self.q_table = self.q_table.append(
                pd.Series(
                    [0]*len(self.actions),
                    index=self.q_table.columns,
                    name=state,
                )
            )

    def choose_action(self, state):

        self.check_state_exist(str(state))

        # action selection
        if np.random.uniform() < self.EPSILON:
            # choose best action
            state_action = self.q_table.loc[state, :]
            # some actions may have the same value, randomly choose on in these actions
            action = np.random.choice(state_action[state_action == np.max(state_action)].index)

        else:
            action = np.random.choice(self.actions)
        return action
    def learn(self, state, action, reward, next_state):
        self.check_state_exist(next_state)
        q_predict = self.q_table.loc[state, action]
        if next_state != 'terminal':
            q_target = reward + self.GAMMA * self.q_table.loc[next_state, :].max()  # next state is not terminal
        else:
            q_target = reward  # next state is terminal
        self.q_table.loc[state, action] += self.ALPHA * (q_target - q_predict)  # update
        self.cost=q_target - q_predict
        self.learn_step=self.learn_step+1
        if self.learn_step % 5 ==0:
            self.costall.append(self.cost)
    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.figure(1)
        plt.plot(np.arange(len(self.costall)),self.costall)
        plt.ylabel("cost")
        plt.xlabel("learning steps")
        plt.savefig("cost.png")

if __name__ == "__main__":
    main = controller(sys.argv[1:])

    main.loop()




