#!/usr/bin/env python2.7

import sys,os, os.path

import rospy
import signal

import math

from amp_final.msg import *
from amp_final.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from plane_functions import *
from rrt import *
import numpy as np
import time
import cv2
import copy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError


class SimulationWindow(object):

	def __init__(self):
		super(SimulationWindow,self).__init__()
		rospy.init_node('controller')
		self.msg = RobotState()
		rate = rospy.Rate(10) # 10hz


		self._dem_item = None
		self._goalIcon = None
		self.gworld = [0,0]
		self.hazmapItem = None
		self._goalID = 'Start'
		self._robotIcon = None
		self.demDownsample = 4
		self.paths = None
		self.count = 0
		self.span = None
		self.goal_titles = ()
		self.row = ()



		self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
		self.steer_sub = rospy.Subscriber('current_steer', Steering, self.state_callback)
		self.goal_sub = rospy.Subscriber('current_goal', NamedGoal, self.goal_cb)
		self.policy_sub = rospy.Subscriber('policy', Policy, self.policy_cb)
		self.current_state_pub = rospy.Publisher('state', RobotState, queue_size=10)
		self.hazard_pub = rospy.Publisher('hazard', Hazard, queue_size=10)


		# self.populated = False

		self.a = 255*.3
		self._goalLocations = [(0,0)]


		self.subproblem = 3

	def dynamicControllerMCTS(self):
		#obstacle_map,size_x,size_y,nodes,r,p_goal,start,goal,tol

		tol = 20
		p_goal = 0.05
		nodes = 40
		r = 40
		scale = 0.005
		path = []
		nodes = []

		start = self.allGoalsDict[self.allGoals[self.count-1][0]]
		goal = (self._goal[1],self._goal[2],0)
		size_x = [0, 4097]
		size_y = [0, 4097]
		size_theta = [0,2*math.pi]
		obstacle_map = self.obstacle_map

		print 'Running RRT'
		#To make plot for only RRT
		#rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,(start.x,start.y),goal,tol)
		#plotMDP(obstacle_map,rrt_nodes,(start.x,start.y),goal,'Navigation Problem Using Only RRT')


		#Make algorithm to put goal at policy recommendation
		current_state = (int(start.y*scale), int(start.x*scale))
		goal_state = (int(goal[1]*scale), int(goal[0]*scale))
		current_location = (start.x,start.y,0)

		while distance([current_location,0], [goal, 0]) > tol:
			desired_state,steer = self.find_desired_stateMCTS(current_state, current_location, goal_state)

			if desired_state == goal_state:
				iter_goal = goal
			else:
				iter_goal = self.find_desired_goal(desired_state)
				iter_goal = (iter_goal[0],iter_goal[1],0)

			print 'Starting at: ' + str(current_location) + ' at ' + str(current_state)
			print 'Navigating to: ' + str(iter_goal) + ' at ' + str(desired_state)
			print 'Driving in direction: ' + str(steer)

			rrt_nodes, pathList = kinematic_rrt(obstacle_map,size_x,size_y,size_theta,nodes,r,p_goal,current_location,iter_goal,tol)
			#plotIterDynMDP(obstacle_map, rrt_nodes, pathList, current_location, iter_goal, goal,self._goalID, 'Iterative Low Level First Order RRT Guided by MCTS')
			path.append(find_pathDyn(rrt_nodes,pathList,current_location,iter_goal))
			nodes.append(rrt_nodes)

			current_location = rrt_nodes[len(rrt_nodes)-1][0]
			current_state = (int(current_location[1]*scale), int(current_location[0]*scale))

			'''self.msg.pose.position.x = current_location[0]
			self.msg.pose.position.y = current_location[1]
			self.current_state_pub.publish(self.msg)'''

		#Plot Path
		print ('Reached Goal')
		plotOverallDynMDP(obstacle_map,nodes,path,(start.x,start.y),goal,self._goalID, 'High Level MCTS Guided First Order RRT')

	def dynamicController(self):
		#obstacle_map,size_x,size_y,nodes,r,p_goal,start,goal,tol

		tol = 20
		p_goal = 0.05
		nodes = 40
		r = 40
		scale = 0.005
		path = []
		nodes = []
		solution_time = []
		control_time = []

		start = self.allGoalsDict[self.allGoals[self.count-1][0]]
		goal = (self._goal[1],self._goal[2],0)
		size_x = [0, 4097]
		size_y = [0, 4097]
		size_theta = [0,2*math.pi]
		obstacle_map = self.obstacle_map

		print 'Running RRT'
		#To make plot for only RRT
		#rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,(start.x,start.y),goal,tol)
		#plotMDP(obstacle_map,rrt_nodes,(start.x,start.y),goal,'Navigation Problem Using Only RRT')


		#Make algorithm to put goal at policy recommendation
		current_state = (int(start.y*scale), int(start.x*scale))
		goal_state = (int(goal[1]*scale), int(goal[0]*scale))
		current_location = (start.x,start.y,0)

		while distance([current_location,0], [goal, 0]) > tol:
			desired_state,steer = self.find_desired_state(current_state, current_location)

			if desired_state == goal_state:
				iter_goal = goal
			else:
				iter_goal = self.find_desired_goal(desired_state)
				iter_goal = (iter_goal[0],iter_goal[1],0)

			print 'Starting at: ' + str(current_location) + ' at ' + str(current_state)
			print 'Navigating to: ' + str(iter_goal) + ' at ' + str(desired_state)
			print 'Driving in direction: ' + str(steer)

			t1 = time.time()
			rrt_nodes, pathList, controls = kinematic_rrt(obstacle_map,size_x,size_y,size_theta,nodes,r,p_goal,current_location,iter_goal,tol)
			t2 = time.time()
			#plotIterDynMDP(obstacle_map, rrt_nodes, pathList, current_location, iter_goal, goal,self._goalID, 'High Level MDP with Low Level Kinematic RRT')
			path.append(find_pathDyn(rrt_nodes,pathList,current_location,iter_goal))
			nodes.append(rrt_nodes)

			current_location = rrt_nodes[len(rrt_nodes)-1][0]
			current_state = (int(current_location[1]*scale), int(current_location[0]*scale))
			solution_time.append(t2 - t1)
			control_time.append(controls[1])
			'''self.msg.pose.position.x = current_location[0]
			self.msg.pose.position.y = current_location[1]
			self.current_state_pub.publish(self.msg)'''

		#Plot Path
		print ('Reached Goal')
		print solution_time
		print control_time
		plotOverallDynMDP(obstacle_map,nodes,path,(start.x,start.y),goal,self._goalID, 'High Level MDP with Low Level Kinematic RRT')

	def controller(self):
		#obstacle_map,size_x,size_y,nodes,r,p_goal,start,goal,tol

		tol = 15
		p_goal = 0.05
		nodes = 5000
		r = 10
		scale = 0.005
		path = []
		nodes = []

		start = self.allGoalsDict[self.allGoals[self.count-1][0]]
		goal = (self._goal[1],self._goal[2])
		size_x = [0, 4097]
		size_y = [0, 4097]
		obstacle_map = self.obstacle_map

		print 'Running RRT'
		#To make plot for only RRT
		#rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,(start.x,start.y),goal,tol)
		#plotMDP(obstacle_map,rrt_nodes,(start.x,start.y),goal,'Navigation Problem Using Only RRT')


		#Make algorithm to put goal at policy recommendation
		current_state = (int(start.y*scale), int(start.x*scale))
		goal_state = (int(goal[1]*scale), int(goal[0]*scale))
		current_location = (start.x,start.y)

		while distance([current_location,0], [goal, 0]) > tol:
			desired_state,steer = self.find_desired_state(current_state, current_location)

			if desired_state == goal_state:
				iter_goal = goal
			else:
				iter_goal = self.find_desired_goal(desired_state)

			print 'Starting at: ' + str(current_location) + ' at ' + str(current_state)
			print 'Navigating to: ' + str(iter_goal) + ' at ' + str(desired_state)
			print 'Driving in direction: ' + str(steer)

			rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,current_location,iter_goal,tol)
			plotIterMDP(obstacle_map,rrt_nodes,current_location,iter_goal,goal,self._goalID, 'Iterative RRT via MDP Solved with VI')
			path.append(find_path(rrt_nodes,current_location,iter_goal))
			nodes.append(rrt_nodes)

			current_location = rrt_nodes[len(rrt_nodes)-1][0]
			current_state = (int(current_location[1]*scale), int(current_location[0]*scale))

			'''self.msg.pose.position.x = current_location[0]
			self.msg.pose.position.y = current_location[1]
			self.current_state_pub.publish(self.msg)'''

		#Plot Path
		print ('Reached Goal')
		plotOverallMDP(obstacle_map,nodes,path,(start.x,start.y),goal,self._goalID, 'Iterative RRT via MDP Solved with VI')

	def controllerMCTS(self):
		#obstacle_map,size_x,size_y,nodes,r,p_goal,start,goal,tol

		tol = 15
		p_goal = 0.05
		nodes = 5000
		r = 10
		scale = 0.005
		path = []
		nodes = []

		start = self.allGoalsDict[self.allGoals[self.count-1][0]]
		goal = (self._goal[1],self._goal[2])
		size_x = [0, 4097]
		size_y = [0, 4097]
		obstacle_map = self.obstacle_map

		print 'Running RRT'
		#To make plot for only RRT
		#rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,(start.x,start.y),goal,tol)
		#plotMDP(obstacle_map,rrt_nodes,(start.x,start.y),goal,'Navigation Problem Using Only RRT')


		#Make algorithm to put goal at policy recommendation
		current_state = (int(start.y*scale), int(start.x*scale))
		goal_state = (int(goal[1]*scale), int(goal[0]*scale))
		current_location = (start.x,start.y)

		while distance([current_location,0], [goal, 0]) > tol:
			desired_state,steer = self.find_desired_stateMCTS(current_state, current_location, goal_state)

			if desired_state == goal_state:
				iter_goal = goal
			else:
				iter_goal = self.find_desired_goal(desired_state)

			print 'Starting at: ' + str(current_location) + ' at ' + str(current_state)
			print 'Navigating to: ' + str(iter_goal) + ' at ' + str(desired_state)
			print 'Driving in direction: ' + str(steer)

			rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,current_location,iter_goal,tol)
			plotIterMDP(obstacle_map,rrt_nodes,current_location,iter_goal,goal,self._goalID, 'Iterative RRT via MDP Guided by MCTS')
			path.append(find_path(rrt_nodes,current_location,iter_goal))
			nodes.append(rrt_nodes)

			current_location = rrt_nodes[len(rrt_nodes)-1][0]
			current_state = (int(current_location[1]*scale), int(current_location[0]*scale))

			'''self.msg.pose.position.x = current_location[0]
			self.msg.pose.position.y = current_location[1]
			self.current_state_pub.publish(self.msg)'''

		#Plot Path
		print ('Reached Goal')
		plotOverallMDP(obstacle_map,nodes,path,(start.x,start.y),goal,self._goalID, 'Iterative RRT via MDP Solved with MCTS')

	def getGoals_client(self):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetGoalList', GetGoalList)
			response = goal()
			row = response.goals

			return response.ids, row
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getAction_client(self,x,y):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetAction', GetAction)
			response = goal(x,y)
			act = response.act

			return act
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def getActionMCTS_client(self,x1,y1,x2,y2,goalID):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetActionMCTS', GetActionMCTS)
			response = goal(x1,y1,x2,y2,goalID)
			act = response.act

			return act
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setCurrentGoal_client(self,id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/SetCurrentGoal', SetCurrentGoal)

			response = goal(id)
			return response.goal
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def state_callback(self, data):			
		self.steer = data.steer
		self._updateRobot()

	def find_desired_state(self,current_state, current_location):
		steer = self.getAction_client(current_location[0],current_location[1])

		if(steer == 1):
			desired_state = (current_state[0]-1,current_state[1])
		elif(steer == 2):
			desired_state = (current_state[0]-1,current_state[1]+1)
		elif(steer == 3):
			desired_state = (current_state[0],current_state[1]+1)
		elif(steer == 4):
			desired_state = (current_state[0]+1,current_state[1]+1)
		elif(steer == 5):
			desired_state = (current_state[0]+1,current_state[1])
		elif(steer == 6):
			desired_state = (current_state[0]+1,current_state[1]-1)
		elif(steer == 7):
			desired_state = (current_state[0],current_state[1]-1)
		elif(steer == 8):
			desired_state = (current_state[0]-1,current_state[1]-1)
		else:
			#We've arrived at the goal - don't publish a new steer
			return

		return desired_state, steer

	def find_desired_stateMCTS(self,current_state, current_location, goal_state):
		steer = self.getActionMCTS_client(current_state[0],current_state[1], goal_state[0],goal_state[1],self._goalID)

		if(steer == 1):
			desired_state = (current_state[0]-1,current_state[1])
		elif(steer == 2):
			desired_state = (current_state[0]-1,current_state[1]+1)
		elif(steer == 3):
			desired_state = (current_state[0],current_state[1]+1)
		elif(steer == 4):
			desired_state = (current_state[0]+1,current_state[1]+1)
		elif(steer == 5):
			desired_state = (current_state[0]+1,current_state[1])
		elif(steer == 6):
			desired_state = (current_state[0]+1,current_state[1]-1)
		elif(steer == 7):
			desired_state = (current_state[0],current_state[1]-1)
		elif(steer == 8):
			desired_state = (current_state[0]-1,current_state[1]-1)
		else:
			#We've arrived at the goal - don't publish a new steer
			return

		return desired_state, steer

	def find_desired_goal(self,desired_state):
		grid_size = float(4097)/float(20)
		state = (desired_state[1]*grid_size+ 0.5*grid_size, desired_state[0]*grid_size+ 0.5*grid_size)
		return state

	def convertToGridCoords(self,i, width, height):
		y = i//width
		x = i % width
		return x, y


	def _updateGoal(self):

		self.allGoals = zip(self.goal_titles, self.row) #Zip the tuples together so I get a list of tuples (instead of a tuple of lists)
		self.allGoals = sorted(self.allGoals, key=lambda param: param[0]) #Sort the combined list by the goal ID
		self.allGoalsDict = dict(self.allGoals)
		
		self._goalID = self.allGoals[self.count][0]


		self.setCurrentGoal_client(self._goalID)	

		self.msg.pose.position.x = self.allGoalsDict[self.allGoals[self.count-1][0]].x
		self.msg.pose.position.y = self.allGoalsDict[self.allGoals[self.count-1][0]].y
		self.current_state_pub.publish(self.msg)

		#Update the label's text:
		#self._goalIcon.setText(str(self._goalID))

		#Pick up the world coordinates
		rospy.wait_for_message('current_goal',NamedGoal)
		self._goalLocations = [self._goal[1], self._goal[2]]

		world = list(copy.deepcopy(self._goalLocations))

		#iconBounds = self._goalIcon.boundingRect()

		self.current_goal_location = world
		world[0] = world[0]/self.demDownsample
		world[1] = world[1]/self.demDownsample
		
		#Adjust the world coords so that the icon is centered on the goal
		#self.gworld[0] = world[0] - iconBounds.width()/2 
		#self.gworld[1] = world[1] - iconBounds.height()/2 #mirror the y coord

		#        world[1] = self.h - (world[1] + iconBounds.height()/2) #mirror the y coord
		#        print 'Ymax:', self.h
		print 'Drawing goal ', self._goalID, ' at ', world


		#self._goalIcon.setPos(QPointF(self.gworld[0], self.gworld[1]))
		self._updateRobot()
		#if self.count > 0:
		#	self.controller()

		

	def _update(self):

		scale = 0.5
		print self.w, self.h
		
		#Overlay the hazmap now that the dem is loaded
		self.hazmap_sub = rospy.Subscriber('hazmap', Image, self.hazmap_cb)


		#Get Goal Prepped ------------------------------------------------------
		titles, row = self.getGoals_client()

		self.goal_titles = self.goal_titles + tuple(titles)
		self.row = self.row + tuple(row)

		self._updateGoal()
		#self.beliefOpacitySlider.valueChanged.connect(self.sliderChanged);
		



	def _updateRobot(self):
		#Redraw the robot locations
		location = self.allGoalsDict[self.allGoals[self.count-1][0]]

		world = list(copy.deepcopy([location.x,location.y]))

		world[0] = world[0]/self.demDownsample
		world[1] = world[1]/self.demDownsample
		

	def hazmap_cb(self, msg):
		#Unlike the dem, the hazmap is pretty standard - gray8 image
		self.hazmap = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
		self.hazmapImage = QImage(self.hazmap, msg.width, msg.height, QImage.Format_Grayscale8)
		self._updateHazmap()
	

	def _updateHazmap(self):
		print 'Rendering hazmap'
		self.obstacle_map = []

		hazTrans = QImage(self.hazmapImage.width(), self.hazmapImage.height(), QImage.Format_ARGB32)
		#hazTrans.fill(Qt.transparent)
		grid_size = float(4097)/float(20)

		for row in range(0, self.hazmapImage.height()):
			for col in range(0, self.hazmapImage.width()):
				#Change the colormap to be clear for clear areas, red translucent for obstacles

				pixColor = self.hazmap[row,col]

				if pixColor == 0:
					#hazTrans.setPixelColor(col, row, QColor(255, 0, 0, 32))
					hazTrans.setPixel(col,row,qRgba(255,0,0,self.a))
					array = np.array([[col*grid_size, (row-1)*grid_size], [col*grid_size, (row)*grid_size], [(col-1)*grid_size, (row)*grid_size], [(col-1)*grid_size, (row-1)*grid_size]])
					self.obstacle_map.append(array+grid_size)
					
				else:
					   #hazTrans.setPixelColor(col, row, QColor(0, 0, 0, 0))
					hazTrans.setPixel(col,row,qRgba(255,255,255,0))


		#self.hazmapItem = self.minimapScene.addPixmap(QPixmap.fromImage(hazTrans)) #.scaled(self.w*100,self.h*100))
		#self.hazmapItem.setPos(QPointF(0, 0))
		trans = QTransform()
		#print 'Translating by:', bounds.width()
		trans.scale(self.w/hazTrans.width(),self.h/hazTrans.height())
		#trans.translate(0, -bounds.height())
		#self.hazmapItem.setTransform(trans)
		if self.subproblem == 1:
			self.controller()
		if self.subproblem == 2:
			self.controllerMCTS()
		if self.subproblem == 3:
			self.dynamicController()
		if self.subproblem == 4:
			self.dynamicControllerMCTS()
		
	def dem_cb(self, msg):


		#self.resolution = msg.info.resolution
		self.w = msg.width
		self.h = msg.height

		print 'Got DEM encoded as:', msg.encoding
		print 'message length:', len(msg.data), 'type:', type(msg.data)
		print 'width:', msg.width
		print 'height:', msg.height
		
		a = np.array(struct.unpack('<%dd' % (msg.width*msg.height), msg.data), dtype=np.float64, copy=False, order='C')

   
		rawDEM = a.reshape((self.h, self.w))
		rawDEM = cv2.resize(rawDEM, (self.h//self.demDownsample, self.w//self.demDownsample), interpolation = cv2.INTER_LINEAR)
		self.h = rawDEM.shape[0]
		self.w = rawDEM.shape[1]

		#Scale to a 8-bit grayscale image:
		self.grayDEM = np.zeros(rawDEM.shape, dtype=np.uint8)
		minZ = np.min(np.min(rawDEM))
		maxZ = np.max(np.max(rawDEM))
		dynRange = maxZ - minZ

		#print 'Max Z:', maxZ
		#print 'Min Z:', minZ
		
		for i in range(0, self.h):
			for j in range(0, self.w):
				self.grayDEM[i][j] = (rawDEM[i][j] - minZ) * 255/dynRange

		#use OpenCV2 to interpolate the dem into something reasonably sized
		print 'Grayscale conversion complete'

		#Needs to be a class variable so that at QImage built on top of this numpy array has
		#a valid underlying buffer - local ars
		#self.resizedDEM = cv2.resize(self.grayDEM, (500,500), interpolation = cv2.INTER_LINEAR)

		print 'Image resize complete'
		
		self.h = self.grayDEM.shape[0]
		self.w = self.grayDEM.shape[1]
		image = QImage(self.grayDEM.reshape((self.h*self.w)), self.w, self.h, QImage.Format_Grayscale8)

		self._dem = image       

		self._update()

	def goal_cb(self, msg):
		 #Resolve the odometry to a screen coordinate for display

		worldX = msg.pose.x
		worldY = msg.pose.y

		print 'Got Goal at: ' + str(worldX) + ',' + str(worldY)

		self._goal = [msg.id, worldX, worldY]
		#self.goals_changed.emit()


	def policy_cb(self,msg):
		pass



def main():
		SimulationWindow()
		rospy.spin()


if __name__ == '__main__':
	main()
	try:
		main()
	except rospy.ROSInterruptException:
		pass

