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
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from Location import *
from OA import *

import sys, pickle
import numpy as np
import cv_bridge
import cv2

class PolicyServer(object):
    def __init__(self):
        print 'Starting policy server'
        self.nodeName = 'policy_server'
        rospy.init_node(self.nodeName)
        self.prev_index = None
        self.index = 0

        
        self.polPack0 = self.loadPolicyPackage('0')
        self.polPack1 = self.loadPolicyPackage('1')

        self.polPacks = [self.polPack1,self.polPack0]
        self.polPack = None

        #print self.polPack
        
        #Initialize services
        self.currentSteer = Steering()
        
        self.setGoalSrv = rospy.Service('~SetCurrentGoal', SetCurrentGoal, self.setCurrentGoalByID)
        self.getGoalSrv = rospy.Service('~GetGoalList', GetGoalList, self.getGoalList)
        self.getMCSrv = rospy.Service('~GetMCSims', GetMCSims, self.getMCSims)
        self.getPaths = rospy.Service('~GetPaths', GetPaths, self.getPaths)
        self.getResults = rospy.Service('~GetResults', GetResults, self.getResults)
        self.getBins = rospy.Service('~Bins', Bins, self.Bins)
        self.getPerf = rospy.Service('~GetPerf', GetPerf, self.getPerf)
        self.setHaz = rospy.Service('~SetCurrentHazard', SetCurrentHazard, self.setHazard)

        
        #Publish maps and things
        self.demPub = rospy.Publisher('dem', Image, queue_size=10, latch=True)
        self.hazPub = rospy.Publisher('hazmap', Image, queue_size=10, latch=True)
        self.goalPub = rospy.Publisher('current_goal', NamedGoal, queue_size=10, latch=True)
        self.steerPub = rospy.Publisher('current_steer', Steering, queue_size=10, latch=True)
        self.policyPub = rospy.Publisher('policy', Policy, queue_size=10, latch=True)

        #self.publishDEM()
        #self.publishHazmap()
        self.setHazard('name')
       
        #Subscribe to a PoseStamped topic for the current robot position
        self.poseSub = rospy.Subscriber('state', RobotState, self.onNewPose)
        

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
        polFile = rospy.get_param('~policy', None) + index + '.pkl'
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

        actionMap = thePolicy['actionMap']
        #print 'Policy:', actionMap

        #Get the current (scaled) position:
        scaledX = int(msg.pose.position.x * self.polPack['scale'])
        scaledY = int(msg.pose.position.y * self.polPack['scale'])

        #Get the location:
        print 'Checking X:', scaledX, ' Y:', scaledY
        
        steerLoc = actionMap[scaledY][scaledX]

        #Convert to degrees:
        if(steerLoc == Location.Forward):
	    steerAngle = -90.0
	elif(steerLoc == Location.FwdRight):
            steerAngle = -45.0
        elif(steerLoc == Location.Right):
            steerAngle = 0.0
        elif(steerLoc == Location.BackRight):
            steerAngle = 45.0
        elif(steerLoc == Location.Back):
            steerAngle = 90.0
        elif(steerLoc == Location.BackLeft):
            steerAngle = 135.0
        elif(steerLoc == Location.Left):
            steerAngle = 180.0
        elif(steerLoc == Location.FwdLeft):
            steerAngle = -135.0
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

    def getPaths(self, req):
        res = GetPathsResponse()

        #Look for a goal with the given id:
        polSims = self.polPack['policies'][req.id]['MCActions']
        self.index = int((self.scaledY) *20) + (int(self.scaledX))
        print 'Got action matrix of size:' + str(np.array(polSims[self.index]).shape)
        size = np.array(polSims[self.index]).shape
        for i in range(1,size[0]):
            msg = pythonList()
            msg.reward = polSims[self.index][i].pop()
            msg.elements = polSims[self.index][i]
            res.paths.append(msg) #all paths pertaining to this starting location
        return res

    def Bins(self, req):
        self.prev_index = self.index
        res = BinsResponse()
        res.bins = []
        outcome = []
        bins= []
        #Publish histogram bins determined by goal
        polSims = self.polPack['policies'][req.id]['MCSims']
        self.index = int((self.scaledY) *20) + (int(self.scaledX)) 
        rewards = np.sort(polSims[self.index,:])

        ul = [1,0.5,0.1,-0.1,-0.5,-1]
        #Transform into OA's
        for i in range(0,len(rewards)):
            try:    
                outcome.append(outcomeAssessment(rewards,rewards[i]))
            except:
                outcome.append(0)
        

        for j in range(0,len(ul)-1):
            this_bin = []
            for i in range(0,len(outcome)):
                if outcome[i] < ul[j] and outcome[i] > ul[j+1]:
                    this_bin.append(rewards[i])
            bins.append(this_bin)

        for i in range(0,len(bins)):
            if bins[i]:
                res.bins.append(bins[i][0])
            else:
                res.bins.append(0)

        print res.bins        

        #res.bins = [100,200,2000,3000,4000]

        return res

    def getMCSims(self, req):
        res = GetMCSimsResponse()

        #Look for a goal with the given id:
        polSims = self.polPack['policies'][req.id]['MCSims']
        polResults = self.polPack['policies'][req.id]['MCResults']
        self.index = int((self.scaledY) *20) + (int(self.scaledX))

        res.rewards = polSims[self.index,:]
        res.results = polResults[self.index,:]
        return res

    def getPerf(self,req):
        res = GetPerfResponse()
        self.index = int((self.scaledY) *20) + (int(self.scaledX))
        
        polR = self.polPack['policies'][req.id]['perfR']
        polAct = self.polPack['policies'][req.id]['perfActions']

        polAct = polAct[self.index]

        res.actions = polAct[0]
        res.reward = float(polR[self.index,1])
        return res

    def getResults(self,req): 
        res = GetResultsResponse()
        polPack = self.polPack
        if req.temporal == 'past':
            index = self.prev_index
            if req.id not in self.polPack['policies']: #we've moved to another pkl
                polPack = self.polPack_previous

        elif req.temporal == 'present':
            index = self.index

        actions = polPack['policies'][req.id]['actualActions']
        rewards = polPack['policies'][req.id]['actualR']
        results = polPack['policies'][req.id]['actualResults']

        actions = actions[index]

        res.reward = float(rewards[index,1])
        res.actions = actions[0]
        res.result = results[index]

        return res

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
        actionMap_val = [item.value for sublist in thePolicy['actionMap'] for item in sublist]
        actionMap = np.reshape(np.array(actionMap_val), (-1, len(thePolicy['actionMap'][0]))).astype(np.uint8)
        
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
    
