#!/usr/bin/env python2.7

import numpy as np
import math
import matplotlib.pyplot as plt


def rrt_algorithm(obstacle_map,size_x,size_y,nodes,r,p_goal,start,goal,tol):
	rrt_nodes = []
	rrt_nodes.append([start, 1])
	count = 0

	while (count < nodes):
		#Run till goal is reached
		sample = [start,1]
		n = len(rrt_nodes)

		while ((checkCollisionSample(obstacle_map, sample) == 1) or (sample[0] in [item[0] for item in rrt_nodes])):
			rand = np.random.random_sample()
			if rand < p_goal:
				sample = [goal, n+1]
			else:
				x_rand = np.random.random_sample()
				y_rand = np.random.random_sample()
				sample = [(size_x[0]+x_rand*(size_x[1]-size_x[0]), size_y[0]+y_rand*(size_y[1]-size_y[0])), n+1]
			

		guess = 0
		nnCost = 1000000

		for i in range(0,n):
			cost = distance(rrt_nodes[i], sample)
			if cost < nnCost:
				nnCost = cost
				nn = i

		p1 = rrt_nodes[nn][0]
		p2 = sample[0]

		collide = checkCollisionPath(obstacle_map,p1,p2)
		
		if collide == 0:
			guess = rrt_nodes[nn]
			graph_index = nn


		if guess == 0:
			continue

		delta = [float(sample[0][0]-guess[0][0])/float(distance(sample,guess))*r, float(sample[0][1] - guess[0][1])/float(distance(sample,guess))*r]
		new_pt = [(guess[0][0] + delta[0], guess[0][1] + delta[1]), graph_index]

		collide = checkCollisionSample(obstacle_map, new_pt)
		if collide == 0:
			rrt_nodes.append(new_pt)

		#check for goal
		if distance(rrt_nodes[len(rrt_nodes)-1], [goal, 0]) < tol:
			return rrt_nodes

		count = count + 1

	print 'Fail'
	return rrt_nodes


def checkCollisionSample(obstacle_map, sample):
	collide = 0
	x = sample[0][0]
	y = sample[0][1]
	for obstacle in obstacle_map:
		if max(obstacle[:,0]) >= x and min(obstacle[:,0]) <= x and max(obstacle[:,1]) >= y and min(obstacle[:,1]) <= y:
			collide = 1

	return collide

def distance(p1,p2):	#Distance between two nodes
	d = math.sqrt((p2[0][0]-p1[0][0])**2+(p2[0][1]-p1[0][1])**2)
	return d

def checkCollisionPath(obstacle_map,p1,p2):
	collide = 0
	tol = 0.05

	#Line between points
	mr = float(p2[1]-p1[1])/float(p2[0]-p1[0])
	pr = p1

	for obstacle in obstacle_map: #Line between vertices of obstacle
		po = []
		mo = []
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				if float(obstacle[0][0]-obstacle[len(obstacle)-1][0]) == 0:
					mo.append(90000)
				else:
					mo.append(float(obstacle[0][1]-obstacle[len(obstacle)-1][1])/float(obstacle[0][0]-obstacle[len(obstacle)-1][0]))
				po.append(obstacle[len(obstacle)-1])
				break
			if float(obstacle[i+1][0]-obstacle[i][0]) == 0:
				mo.append(90000)
			else:
				mo.append(float(obstacle[i+1][1]-obstacle[i][1])/float(obstacle[i+1][0]-obstacle[i][0]))
			po.append(obstacle[i])


		for j in range(0,len(mo)):
			if mo[j] == 90000:
				x = obstacle[j][0]
				y = mr*(x-pr[0]) + pr[1]
			else:
				x = float(-mo[j]*po[j][0] + po[j][1] + mr*pr[0] - pr[1])/float(mr - mo[j])
				y = mo[j]*(x - po[j][0]) + po[j][1]

			if j == len(mo)-1:
				index = 1
			else:
				index = j + 1

			vec = [obstacle[j][0],obstacle[index][0]]
			vec.sort()
			
			vec_x = [obstacle[j][1], obstacle[index][1]]
			vec_x.sort()

			vec_k = [p1[1], p2[1]]
			vec_k.sort()

			if x >= vec[0] and x <= vec[1]:
				if y >= vec_x[0] and y <= vec_x[1]:
					if y + tol >= vec_k[0] and y - tol <= vec_k[1]:
						collide = 1


	return collide

def find_path(rrt_nodes, start, goal):
	fpath = []
	fpath.append(goal)
	pointer = rrt_nodes[len(rrt_nodes)-1][1]

	while (start in [item for item in fpath]) == 0:
		next_node = rrt_nodes[pointer]
		fpath.append(next_node[0])
		pointer = next_node[1]
	return fpath

def plot(obstacle_map, rrt_nodes, start, goal):
	for item in rrt_nodes:
		plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches
	for i in range(0,len(rrt_nodes)-1):
		index = rrt_nodes[i][1]
		plt.plot([rrt_nodes[i][0][0],rrt_nodes[index][0][0]],[rrt_nodes[i][0][1],rrt_nodes[index][0][1]],'r')

	#Plot path
	path = find_path(rrt_nodes,start,goal)
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],'g')

	plt.show()

def plotMDP(obstacle_map, rrt_nodes, start, goal, title):
	plt.title(title)
	plt.xlabel('X [Units]')
	plt.ylabel('Y [Units]')
	plt.ylim(4097, 0)
	plt.xlim(0, 4097)
	


	for item in rrt_nodes:
		plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches
	for i in range(0,len(rrt_nodes)-1):
		index = rrt_nodes[i][1]
		plt.plot([rrt_nodes[i][0][0],rrt_nodes[index][0][0]],[rrt_nodes[i][0][1],rrt_nodes[index][0][1]],'r')

	#Plot path
	path = find_path(rrt_nodes,start,goal)
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],'g')

	#Make start, end noticable
	plt.plot(start[0], start[1],'g.',markersize=10)
	plt.text(goal[0], goal[1],'A',fontsize=20)

	plt.show()

def plotIterMDP(obstacle_map, rrt_nodes, start, itergoal, goal,goalID, title):
	plt.title(title)
	plt.xlabel('X [Units]')
	plt.ylabel('Y [Units]')
	plt.ylim(4097, 0)
	plt.xlim(0, 4097)
	
	for item in rrt_nodes:
		plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches
	for i in range(0,len(rrt_nodes)-1):
		index = rrt_nodes[i][1]
		plt.plot([rrt_nodes[i][0][0],rrt_nodes[index][0][0]],[rrt_nodes[i][0][1],rrt_nodes[index][0][1]],'r')

	#Plot path
	path = find_path(rrt_nodes,start,itergoal)
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],'g')

	#Make start, end noticable
	plt.plot(start[0], start[1],'g.',markersize=10)
	plt.text(goal[0], goal[1],goalID,fontsize=20)

	plt.show()

def plotOverallMDP(obstacle_map, rrt_nodes, path, start, goal,goalID, title):
	plt.title(title)
	plt.xlabel('X [Units]')
	plt.ylabel('Y [Units]')
	plt.ylim(4097, 0)
	plt.xlim(0, 4097)

	for iterem in rrt_nodes:
		for item in iterem:
			plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches
	for iterem in rrt_nodes:
		for i in range(0,len(iterem)-1):
			index = iterem[i][1]
			plt.plot([iterem[i][0][0],iterem[index][0][0]],[iterem[i][0][1],iterem[index][0][1]],'r')

	#Plot path
	#path = find_path(rrt_nodes,start,goal)
	for iterem in path:
		for i in range(0,len(iterem)-1):
			plt.plot([iterem[i][0],iterem[i+1][0]],[iterem[i][1],iterem[i+1][1]],'g')

	#Make start, end noticable
	plt.plot(start[0], start[1],'g.',markersize=10)
	plt.text(goal[0], goal[1],goalID,fontsize=20)

	plt.show()


def kinematic_rrt():
	# Inputs: velocity, turning angle
	pass


def rk4(initial, fun, h, t):
	n = float(t)/float(h+1)
	states = []
	states.append(initial)

	for i in range(1,len(t)):
		step = (i-1)*h
		k1 = h*fun(step,states[i-1])
		k2 = h
		k3 = h*fun(step,states[i-1])
		k4 = h
		states.append(states[i-1] + float(1)/flaot(6)*(k1 + 2*k2 + 2*k3 + k4))

	return states


if __name__ == '__main__':
	'''O1 = np.array(np.mat('1 1; 2 1; 2 5; 1 5'));
	O2 = np.array(np.mat('3 4; 4 4; 4 12; 3 12'));
	O3 = np.array(np.mat('3 12; 12 12; 12 13; 3 13'));
	O4 = np.array(np.mat('12 5; 13 5; 13 13; 12 13'));
	O5 = np.array(np.mat('6 5; 12 5; 12 6; 6 6'));
	obstacle_map = [O1, O2, O3, O4, O5]
	size_x = (-1,13)
	size_y = (-1,13)
	start = (0,0)
	goal = (10,10)'''

	O1 = np.array(np.mat('-6 -6; 25 -6; 25 -5; -6 -5'));
	O2 = np.array(np.mat('-6 5; 30 5; 30 6; -6 6'));
	O3 = np.array(np.mat('-6 -5; -5 -5; -5 5; -6 5'));
	O4 = np.array(np.mat('4 -5; 5 -5; 5 1; 4 1'));
	O5 = np.array(np.mat('9 0; 10 0; 10 5; 9 5'));
	O6 = np.array(np.mat('14 -5; 15 -5; 15 1; 14 1'));
	O7 = np.array(np.mat('19 0; 20 0; 20 5; 19 5'));
	O8 = np.array(np.mat('24 -5; 25 -5; 25 1; 24 1'));
	O9 = np.array([[29, 0], [30, 0], [30, 5], [29, 5]]);
	obstacle_map = [O1, O2, O3, O4, O5, O6, O7, O8, O9]
	size_x = (-6,36)
	size_y = (-6,6)
	start = (0,0)
	goal = (35,0)

	rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,5000,0.5,0.05,start,goal,0.25)
	plot(obstacle_map,rrt_nodes, start, goal)