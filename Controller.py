#!/usr/bin/python2

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os, os.path

from amp_final.srv import *
from amp_final.msg import *
from sensor_msgs.msg import *

import rospy
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

import cv2
import copy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError



class SimulationWindow():

	def __init__(self):
		#self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
		#self.steer_sub = rospy.Subscriber('current_steer', Steering, self.state_callback)
		#self.goal_sub = rospy.Subscriber('current_goal', NamedGoal, self.goal_cb)
		#self.option_sub = rospy.Subscriber('option', OptionSelect, self.option_cb)
		self.current_state_pub = rospy.Publisher('state', RobotState, queue_size=10)



		rospy.init_node('controller')
		self.msg = RobotState()
		rate = rospy.Rate(10) # 10hz

		print 'sec'


if __name__ == '__main__':
	try:
		SimulationWindow()
	except rospy.ROSInterruptException:
		pass
