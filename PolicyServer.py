#!/usr/bin/python2

'''
ROS node to suck in a policy package pickle and do the following things:
1. Publish the raw DEM source image as a map
2. Publish the hazard map as known to the server (the corrupted one)
3. Publish current goal (if any)

4. Subscribe to Unity/Pose
OnNewMessage: 
Query MDP for current steer based on pose and scale
Publish steer angle

Services: 
GetGoalList (returns ID and coords)
SetCurrentGoalByID

'''

import rospy
from amp_final.srv import *
from amp_final.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from Location import *
from MDPSolver import *
import floyd as floyd
import sys, pickle
import numpy as np
import cv_bridge
import cv2
import os.path

class PolicyServer(object):
	def __init__(self):
		print 'Starting policy server'
		self.nodeName = 'policy_server'
		rospy.init_node(self.nodeName)
		self.prev_index = None
		self.index = 0
		self.current = None

		
		self.polPack0 = self.loadPolicyPackage('0')

		self.polPacks = [self.polPack0]
		self.polPack = None

		#print self.polPack
		
		#Initialize services
		self.currentSteer = Steering()
		
		self.setGoalSrv = rospy.Service('~SetCurrentGoal', SetCurrentGoal, self.setCurrentGoalByID)
		self.getGoalSrv = rospy.Service('~GetGoalList', GetGoalList, self.getGoalList)
		self.getActionSrv = rospy.Service('~GetAction', GetAction, self.getAction)
		self.getActionMCTSSrv = rospy.Service('~GetActionMCTS', GetActionMCTS, self.getActionMCTS)
		
		#Publish maps and things
		self.demPub = rospy.Publisher('dem', Image, queue_size=10, latch=True)
		self.hazPub = rospy.Publisher('hazmap', Image, queue_size=10, latch=True)
		self.goalPub = rospy.Publisher('current_goal', NamedGoal, queue_size=10, latch=True)
		self.steerPub = rospy.Publisher('current_steer', Steering, queue_size=10, latch=True)
		self.policyPub = rospy.Publisher('policy', Policy, queue_size=10, latch=True)

		#self.publishDEM()
		#self.publishHazmap()

	   
		#Subscribe to a PoseStamped topic for the current robot position
		self.poseSub = rospy.Subscriber('state', RobotState, self.onNewPose)
		
		print('Solving Floyd Warshall')
		if os.path.isfile('floydWarshallCosts.npy') == 0:
			self.costmap, nextPlace = floyd.floyds(self.polPack0['hazmap'])
			np.save('floydWarshallCosts', self.costmap)
		else:
			self.costmap = np.load('floydWarshallCosts.npy')


		self.setHazard('name')
		print 'Policy server ready!'

	 
	def publishDEM(self):

		self.demPub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(self.polPack['src'], encoding='64FC1'))
		print 'DEM published'
		
	def publishHazmap(self):
		hazRaw = self.polPack['hazmap']
		#Invert to match ROS convention of white=traversable

		self.hazPub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(cv2.bitwise_not(hazRaw), encoding='mono8'))
		
	def loadPolicyPackage(self, index):
		#Load a policy package from the pickle
		polFile = rospy.get_param('~policy', None)  + '.pkl'
		if polFile is None:
			print 'Parameter policy not defined'
			return None

		print 'Loading policy package from:', polFile
		pFile = open(polFile, 'rb')
		return pickle.load(pFile)
	
	def onNewPose(self, msg):
		#Query the policy for the current goal with the current position and publish the appropriate
		#action extracted from the policy
		
		#no goal has been set yet
		if self.currentSteer.id == '':
			return
		
		thePolicy = self.polPack['policies'][self.currentSteer.id]

		actionMap = thePolicy['actionMapClean']
		#print 'Policy:', actionMap

		#Get the current (scaled) position:
		scaledX = int(msg.pose.position.x * self.polPack['scale'])
		scaledY = int(msg.pose.position.y * self.polPack['scale'])

		#Get the location:
		print 'Checking X:', scaledX, ' Y:', scaledY
		
		steerLoc = actionMap[scaledY][scaledX]

		#Convert to degrees:
		if(steerLoc == Location.Forward):
			steerAngle = 1
		elif(steerLoc == Location.FwdRight):
			steerAngle = 2
		elif(steerLoc == Location.Right):
			steerAngle = 3
		elif(steerLoc == Location.BackRight):
			steerAngle = 4
		elif(steerLoc == Location.Back):
			steerAngle = 5
		elif(steerLoc == Location.BackLeft):
			steerAngle = 6
		elif(steerLoc == Location.Left):
			steerAngle = 7
		elif(steerLoc == Location.FwdLeft):
			steerAngle = 8
		else:
			#We've arrived at the goal - don't publish a new steer
			return

		self.scaledX = scaledX
		self.scaledY = scaledY
		self.currentSteer.steer = int(steerAngle)
		self.currentSteer.header.stamp = rospy.Time.now()
		
		self.steerPub.publish(self.currentSteer)
		
	def getGoalList(self, req):
		res = GetGoalListResponse()

		#Publish things out of the polPackage dictionary
		for goalID, policy in self.polPack['policies'].iteritems():
			#print 'Policy:', policy
			
			res.ids.append(goalID)
			polGoal =  policy['goal']
			theGoal = Pose2D(polGoal[1], polGoal[0], 0.0) #goals are stored as (row, col), swap in the message
			res.goals.append(theGoal)
		return res

	def getAction(self,req):
		thePolicy = self.polPack['policies'][self.currentSteer.id]

		actionMap = thePolicy['actionMapClean']
		#print 'Policy:', actionMap

		#Get the current (scaled) position:
		print self.polPack['scale']
		scaledX = int(req.x * self.polPack['scale'])
		scaledY = int(req.y * self.polPack['scale'])

		#Get the location:
		print 'Checking X:', scaledX, ' Y:', scaledY
		
		steerLoc = actionMap[scaledY][scaledX]

		#Convert to degrees:
		if(steerLoc == Location.Forward):
			steerAngle = 1
		elif(steerLoc == Location.FwdRight):
			steerAngle = 2
		elif(steerLoc == Location.Right):
			steerAngle = 3
		elif(steerLoc == Location.BackRight):
			steerAngle = 4
		elif(steerLoc == Location.Back):
			steerAngle = 5
		elif(steerLoc == Location.BackLeft):
			steerAngle = 6
		elif(steerLoc == Location.Left):
			steerAngle = 7
		elif(steerLoc == Location.FwdLeft):
			steerAngle = 8
		else:
			#We've arrived at the goal - don't publish a new steer
			return

		return steerAngle

	def getActionMCTS(self,req):
		if self.current == req.id:
			pass
		else:
			self.ans_clean = self.solveGoal(self.polPack['hazmap'], [req.goalx,req.goaly])
		steerAngle = self.ans_clean.solveMCTS([req.startx,req.starty],self.costmap,[req.goalx,req.goaly],1)
		self.current = req.id

		return steerAngle

	def setCurrentGoalByID(self, req):
		res = SetCurrentGoalResponse()

		#Look for a goal with the given id:
		for policyID in self.polPack['policies']:
			if policyID == req.id:
				polGoal = self.polPack['policies'][policyID]['goal']
				res.goal = Pose2D(polGoal[1], polGoal[0], 0.0)
				break;

		print 'Setting desired goal: ', req.id, ' X:', res.goal.x, ' Y:', res.goal.y
		self.currentSteer.goal = res.goal
		self.currentSteer.id = req.id

		goalMsg = NamedGoal()
		goalMsg.header.stamp = rospy.Time.now()
		goalMsg.pose = res.goal
		goalMsg.id = policyID
		
		self.goalPub.publish(goalMsg)

		#Massage the policy into an unsigned int datatype for ROS
		thePolicy = self.polPack['policies'][self.currentSteer.id]
		actionMap_val = [item.value for sublist in thePolicy['actionMapClean'] for item in sublist]
		actionMap = np.reshape(np.array(actionMap_val), (-1, len(thePolicy['actionMapClean'][0]))).astype(np.uint8)
		
		#Publish the policy for this goal:
		polMsg = Policy()
		polMsg.header.stamp = rospy.Time.now()
		polMsg.goalID = policyID
		polMsg.height = actionMap.shape[0]
		polMsg.width = actionMap.shape[1]
		#Use the CvBridge to help the conversion to uint8
		imgMsg = cv_bridge.CvBridge().cv2_to_imgmsg(actionMap, encoding='8UC1')
		
		polMsg.policy = imgMsg.data 
		self.policyPub.publish(polMsg)
		


		return res

	def solveGoal(self,hazMap, goal):
		ans = MDPSolver(modelName='HazmapModel', hazImg=hazMap, goal=goal);
		ans.solve()
		return ans

	
	def setHazard(self,req):
		if self.polPack:
			self.polPack_previous = self.polPack
		self.polPack = self.polPacks.pop()
		self.publishDEM()
		self.publishHazmap()
	
	def run(self):
		rospy.spin()
		
if __name__ == "__main__":
	server = PolicyServer()
	server.run()
	
