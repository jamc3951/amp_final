#!/usr/bin/python2

'''
######################################################

From Luke's MDP Solver example code
https://github.com/clburks9/EasyPOMDP.git

Author: Luke Burks
Date: April 2017

Implements an MDP on the discrete hallway problem

Written for a guest lecture for Nisar Ahmeds 
Aerospace Algorithms for Autonomy class


######################################################
'''

import copy
import matplotlib.pyplot as plt
import numpy as np
from ModelSpec import *
from Location import *
import time
from treeNode import Node
#from mcts import *
import math
import sys
import pickle
import pdb


class MDPSolver():
	# Try to find the model the user has asked for:
	def __init__(self, modelName=None, *args, **kwds):

		# print 'Args:', args
		# print 'KWArgs:', kwds

		if modelName == None:
			print('MDPSolver: Provide a Python class implementing type ModelSpec')
			return
		print('Using model:', modelName)
		modelModule = __import__(modelName, globals(),
								 locals(), [modelName], 0)

		classes = [c for c in modelModule.__dict__.values()
				   if isinstance(c, type)]
		base_subclasses = [c for c in classes if issubclass(c, ModelSpec)]
		# print 'Model class:', base_subclasses[-1]

		self.model = base_subclasses[-1](*args, **kwds)

	def listComp(self, a, b):
		# Use an epsilon instead of floating point precision
		eps = 0.1e-05
		if(len(a) != len(b)):
			return False
		for i in range(0, len(a)):
			if(math.fabs(a[i] - b[i]) > eps):
				return False

		return True

		def printActionGrid(self, vals):
			print(vals[Location.FwdLeft.value], ' ', vals[Location.Forward.value], ' ', vals[Location.FwdRight.value])
			print(vals[Location.Left.value], ' ', vals[Location.Current.value], ' ', vals[Location.Right.value])
			print(vals[Location.BackLeft.value], ' ', vals[Location.Back.value], ' ', vals[Location.BackRight.value])

	def solve(self):

		#print('Finding Value Function');

		#self.V = [np.min(np.min(self.model.R))]*self.model.N;
		self.V = np.zeros((self.model.N))

		W = [np.random.random()]*self.model.N
		# print 'Start:', np.reshape(self.V, (4,4))
		iterCount = 0

		while(not self.listComp(self.V, W)):
			W = copy.copy(self.V)

			for i in range(0, self.model.N):
				actionValues = np.zeros(self.model.acts)
				for a in range(0, self.model.acts):
					#  transReward = 0.0
					transReward=np.sum(W*self.model.px[a][i][:])
					#  for j in range(0, self.model.N):
					#      transReward += self.model.px[a][i][j]*W[j]
					actionValues[a] = self.model.discount * \
						transReward + self.model.R[a][i]
					# if if leads off map -> actionValues[a] = 0 (or giant negative in the case of negative rewards)
					# i = position, a sends off map
				#  row, col = self.model.getIndex2D(i)

				# print 'State (', row, ',', col, ')'
				# self.printActionGrid(actionValues)

				bestValue = max(actionValues)
				self.V[i] = bestValue

				# print 'Action sum:', sum(self.V[j]*self.model.px[a][i][j] for j in range(0,self.model.N) for a in range(0,self.model.acts))
				# print 'V:', self.V
			#self.V[i] = self.model.discount * max(self.model.R[a][i] + sum(self.V[j]*self.model.px[a][i][j] for j in range(0,self.model.N)) for a in range(0,self.model.acts));
			# print np.reshape(self.V, (5,5))
			# raw_input()
			iterCount += 1
		print('Used ', iterCount, ' iterations')

	def getAction(self, x):
		y = [0]*self.model.acts
		for a in range(0, self.model.acts):
			y[a] = self.model.R[a][x] + \
				np.sum(self.V[j]*self.model.px[a][x][j]
					   for j in range(0, self.model.N))

		row, col = self.model.getIndex2D(x)
		if (row == 0):
			y[Location.Forward.value] = -10000000
			y[Location.FwdLeft.value] = -10000000
			y[Location.FwdRight.value] = -10000000
		if (row == 19):
			y[Location.Back.value] = -10000000
			y[Location.BackLeft.value] = -10000000
			y[Location.BackRight.value] = -10000000
		if (col == 19):
			y[Location.Right.value] = -10000000
			y[Location.FwdRight.value] = -10000000
			y[Location.BackRight.value] = -10000000
		if (col == 0):
			y[Location.Left.value] = -10000000
			y[Location.FwdLeft.value] = -10000000
			y[Location.BackLeft.value] = -10000000

		act = np.argmax(y)
		return self.model.getAction(act)

	def getActionMap(self):
		grid = np.zeros((self.model.height, self.model.width)).tolist()

		for i in range(0, self.model.N):
			[x, y] = convertToGridCoords(
				i, self.model.width, self.model.height)
			grid[y][x] = self.getAction(i)
		return grid



	def perfSample(self, startLoc, actions):
		start = 20*startLoc[0] + startLoc[1]
		reward=self.model.R_values[start]
		probabilties = copy.copy(self.model.px)
		action_list = []
		possible_transitions = []
		previous_locations = []
		current=copy.copy(start)
		previous_locations.append(current)
		action_list.append(float(current))
		while self.model.R_values[current] not in [self.model.obstacleReward,self.model.goalReward]:
			[x,y] = convertToGridCoords(current,self.model.width,self.model.height)
			act = actions[y][x].value
			#look at indeces w transition probabilties, find maximum likelihood, verify its no obstacle (if is then move to next)
			possible_transitions = []
			go_r = 0


			thing = np.argmax(probabilties[act][current][:])

			#print self.model.px[act][current][:]
			for i in range(self.model.N):
				if self.model.R_values[i] == self.model.obstacleReward or i in previous_locations:
				   probabilties[act][current][i] = 0
			#print self.model.px[act][current][:]

			previous_locations.append(current)
			current = np.argmax(probabilties[act][current][:])
			if current == 0:
				#print 'we stuck'
				return action_list, reward
			previous_locations.append(current)

			action_list.append(float(current))
			reward+=self.model.R_values[current]

			'''if self.model.R_values[current] == self.model.goalReward:
				return action_list, reward

			else:
				return start'''
		return action_list, int(reward+ 3*self.model.stationaryReward)

	def MCSample(self, startLoc, actions):
		start = 20*startLoc[0] + startLoc[1]
		reward=self.model.R_values[start]
		action_list = []
		result = 0
		current=copy.copy(start)
		action_list.append(float(current))
		while self.model.R_values[current] not in [self.model.obstacleReward,self.model.goalReward]:
			[x,y] = convertToGridCoords(current,self.model.width,self.model.height)
			act = actions[y][x].value
			current = np.random.choice(range(self.model.N),p=self.model.px[act][current][:])
			action_list.append(float(current))
			reward+=self.model.R_values[current]
			if self.model.R_values[current] == self.model.goalReward:
				result = 1

		action_list.append(int(reward))
		return int(reward), action_list, result

	def ActualSample(self, startLoc, actions):
		start = 20*startLoc[0] + startLoc[1]
		reward=self.model.R_values[start]
		action_list = []
		result = 0
		current=copy.copy(start)
		action_list.append(float(current))
		while self.model.R_values[current] not in [self.model.obstacleReward,self.model.goalReward]:
			[x,y] = convertToGridCoords(current,self.model.width,self.model.height)
			act = actions[y][x].value
			current = np.random.choice(range(self.model.N),p=self.model.px[act][current][:])
			action_list.append(float(current))
			reward+=self.model.R_values[current]

			if self.model.R_values[current] == self.model.goalReward:
				result = 1

		return int(reward), action_list, result


	def solveMCTS(self,start,costmap,goalLoc,MC):
		self.costmap = costmap
		#Worst case scenario: decrease exploration, increase tree depth (may not work)
		self.gamma = .9; 

		#Number of Actions
		self.numActions = self.model.acts;

		#Max solver time per action
		self.maxTime = 1; #max time I spend searching

		#Exploration Constant
		self.c = 1;  
		self.goal = 20*goalLoc[0] + goalLoc[1]

		print ('Goal:' , convertToGridCoords(self.goal,20,20))
		#Starting state
		state = 20*start[0] + start[1]; 

		print ('Start:' , convertToGridCoords(state,20,20))
		#Set max depth
		maxDepth = 100; #breadth vs depth


		state_list = []
		reward = 0
		result = 0;
		count = 0; 
		while(count < 60):

			h = Node(); 
			act = self.search(state,h,maxDepth); #fake
			state = self.generate_s(state, act); #actually move transition[h]
			r = self.generate_r(state); #generate reward for that fake action
			#print("State: {}, Action Taken: {}, Reward Gained: {}".format(convertToGridCoords(state,20,20),act,r));

			x,y = convertToGridCoords(state,self.model.width,self.model.height)

			state_list.append(float(state))
			reward = reward + r
			if goalLoc[0] == y and goalLoc[1] == x:
				result = 1;
				print("Goal Reached"); 
				if MC == 1:
					state_list.append(int(reward))
				return reward,state_list,result


			if self.isTerminal(state) == True:
				print 'I have failed and I know it'
				if MC == 1:
					state_list.append(int(reward))
				return reward,state_list,result

			count = count + 1
		if MC == 1:
			state_list.append(int(reward))
		return reward,state_list,result
	def simulate(self, s, h, depth): #fake 


		# check if node is in tree
		# if not, add nodes for each action
		if(depth <= 0):
			return 0

		#Add all possible actions to nodes children
		if(len(h.children) == 0): 
			for a in range(0,self.numActions):
				h.addChildID(a)
				h[a].N = 1; 


		# find UCT algorithms suggested action
		act = np.argmax([ha.Q + self.c*np.sqrt(np.log(h.N)/ha.N) for ha in h])

		# generate s,r
		sprime = self.generate_s(s,act)
		r = self.generate_r(sprime) #allowed to change

		if self.isTerminal(sprime) == True:
			return r #maybe update nodes


		# # if this is the first time you've seen this node
		if(len(h[act].children) == 0):
			for a in range(0,self.numActions):
				h[act].addChildID(a); 
				h[act][a].N = 1;
			#return self.rollout(s,depth); #Choice here
			return self.estimate_value(s);

			#return r which r is it 
		q = r + self.gamma * \
			self.simulate(sprime, h[act], depth-1)

		# update node values
		h.N += 1
		h[act].N += 1
		h[act].Q += (q-h[act].Q)/h[act].N


		return q


	def search(self, s, h, depth):
		count = 0

		startTime = time.clock()

		#while(time.clock()-startTime < self.maxTime): 
		while(count < 1000):
			count += 1
			self.simulate(s, h, depth)
		#print([a.Q for a in h]); 
		return np.argmax([a.Q for a in h]) #approximation of value function


	def generate_s(self,s,act): #random choice with state and action
		current = np.random.choice(range(self.model.N),p=self.model.px[act][s][:])

		return current; 

	def generate_r(self,s): #query new reward
		#current = np.random.choice(range(self.model.N),p=self.model.px[act][s][:])
		return self.model.R_values[s]


	def isTerminal(self,s): #check if we go to goal or obstacle
		if self.model.R_values[s] in [self.model.obstacleReward, self.model.goalReward]:
			return True;
		else:
			return False;

	def rollout(self,s,depth): #rollouts beyond known tree
		goal = convertToGridCoords(self.goal,self.model.width,self.model.height)
	
		if(depth <= 0):
			return 0

		# obstacle aware greedy heuristic 
		guess = []
		for a in range(0,self.numActions):
			state = self.generate_s(s,a)
			if self.generate_r(state) == self.model.obstacleReward:
				guess.append(-10000)
			else:
				state_grid = convertToGridCoords(state,self.model.width,self.model.height)
				d2g = self.costmap[goal[0]][goal[1]][state_grid[0]][state_grid[1]]
				if d2g == np.inf or d2g == -np.inf:
					guess.append(self.model.obstacleReward*5)
				else:
					guess.append(self.model.goalReward - np.abs(d2g*self.model.stationaryReward))
			#exclude states that hit obtacles
			#is generate r negative?: exclude


		act = np.argmax(guess)
		# random
		#act = np.random.randint(0,8)
		sprime = self.generate_s(s,act)
		r = self.generate_r(sprime)

		return r + self.gamma*self.rollout(sprime, depth-1)

	def estimate_value(self,s): #Hack: this node you saw for the first time, is as valuble as 10-dist2goal (expected discounted rewards)
		goal = convertToGridCoords(self.goal,self.model.width,self.model.height)
		s = convertToGridCoords(s,self.model.width,self.model.height)
		d2g = self.costmap[goal[0]][goal[1]][s[0]][s[1]] #double flip this is correct

		if d2g == np.inf or d2g == -np.inf:
			return self.model.obstacleReward*5
		if d2g == 0.0:
			return self.model.goalReward*10
		else:
			return (self.model.goalReward - np.abs(d2g*self.model.stationaryReward)) #+self.model.goalReward; #Floyd-Warshall: what is the expected dist2goal based on sparse planning

def checkValue(ans):
	plt.plot(ans.V)
	plt.title('Value Function')
	plt.show()


def checkPolicy(ans):
	colorGrid = ['g', 'r', 'b']
	grid = np.ones(shape=(1, ans.model.N))
	for i in range(1, ans.model.N):
		print(ans.getAction(i))
	grid[0][i] = ans.getAction(i)
	plt.imshow(grid, extent=[0, ans.model.N-1, 0, 1], cmap='inferno')
	plt.title('Policy Implementation')
	plt.show()


def checkBoth(ans):
	fig, axarr = plt.subplots(2, sharex=True)
	axarr[0].plot(ans.V, linewidth=5)
	axarr[0].legend(['Value Function'])

	colorGrid = ['g', 'r', 'b']
	grid = np.ones(shape=(1, ans.model.N))
	for i in range(1, ans.model.N):
		grid[0][i] = ans.getAction(i)
	axarr[1].scatter(-5, .5, c='k')
	axarr[1].scatter(-5, .5, c='r')
	axarr[1].scatter(-5, .5, c='y')

	axarr[1].imshow(grid, extent=[0, ans.model.N-1, 0, 1], cmap='inferno')
	axarr[1].set_xlim([0, 20])
	axarr[1].set_title('Policy Implementation')
	axarr[1].legend(['Left', 'Right', 'Stay'])
	plt.suptitle('Hallway MDP')

	plt.show()


def convertToGridCoords(i, width, height):
	y = i//width
	x = i % width
	return x, y


def convertToGrid(a, width, height):
	return np.reshape(a, (height, width))


	#  b = np.zeros(shape=(height,width))
	#  for i in range(0,height*width):
	#          b[i//10][i%10]=a[i];
	#  return b;


def checkBoth2D(ans, hazMap):
	print('Got:', np.reshape(ans.V, (ans.model.width, ans.model.height)))
	Vtilde = convertToGrid(ans.V, ans.model.width, ans.model.height)

	grid = np.ones((ans.model.height, ans.model.width)).tolist()

	for i in range(0, ans.model.N):
		[x, y] = convertToGridCoords(i, ans.model.width, ans.model.height)
		grid[y][x] = ans.getAction(i)

		print(grid)

		arrowGridAngle = np.zeros((ans.model.height, ans.model.width))

	for y in range(0, ans.model.width):
		for x in range(0, ans.model.height):
			if(grid[y][x] == Location.Forward):
				arrowGridAngle[y][x] = 90.0
			elif(grid[y][x] == Location.FwdRight):
				arrowGridAngle[y][x] = 45.0
			elif(grid[y][x] == Location.Right):
				arrowGridAngle[y][x] = 0.0
			elif(grid[y][x] == Location.BackRight):
				arrowGridAngle[y][x] = 315.0
			elif(grid[y][x] == Location.Back):
				arrowGridAngle[y][x] = 270.0
			elif(grid[y][x] == Location.BackLeft):
				arrowGridAngle[y][x] = 225.0
			elif(grid[y][x] == Location.Left):
				arrowGridAngle[y][x] = 180.0
			elif(grid[y][x] == Location.FwdLeft):
				arrowGridAngle[y][x] = 135.0

	X, Y = np.mgrid[0:ans.model.height:1, 0:ans.model.width:1]

	#U = np.cos(X);
	#V = np.sin(Y);

	#obstacles = [[4,4],[7,2],[7,7],[2,5]];
	#obsX = [4,7,7,2];
	#obsY = [4,2,7,5];
	plt.contourf(Vtilde, alpha=0.5)
	# plt.scatter(obsX,obsY,c='r',s=150,marker='8');

	plt.quiver(X, Y, angles=arrowGridAngle, color='k')
	plt.scatter(ans.model.goal[1], ans.model.goal[0], c='c', s=250, marker='*')

	plt.imshow(hazMap, alpha=0.5, cmap='binary')
	plt.title('MDP Value Function and Policy')
	plt.ylim(max(plt.ylim()), min(plt.ylim()))

	plt.show()


if __name__ == "__main__":

	if len(sys.argv) < 2:
		print('usage: ', sys.argv[0], ' <hazmap.npy> <scale>')
		sys.exit(0)
	hazMap = np.load(sys.argv[1])
	scale = float(sys.argv[2])
	goal = (1, 3)  # specified in x,y

	ans = MDPSolver(modelName='HazmapModel', hazImg=hazMap, goal=goal)
	ans.solve()

	# checkValue(ans);
	# checkPolicy(ans);
	# checkBoth(ans);
	checkBoth2D(ans, hazMap)

	# Save the policy pickle for further processing:
	# Include the goal, policy, hazmap, and scale factor

	dataset = {'policy': ans,
			   'goal': goal,
			   'hazmap': hazMap,
			   'scale': scale}
	output = open('policy_%1.3f.pkl' % scale, 'wb')
	pickle.dump(dataset, output)
