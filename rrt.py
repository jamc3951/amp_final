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
				if float(mr - mo[j]) == 0:
					x = 900000
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
	plt.grid()
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
	plt.grid()
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


def kinematic_rrt(obstacle_map,size_x,size_y,size_theta,nodes,r,p_goal,start,goal,tol):
	rrt_nodes = []
	pathList = []
	rrt_nodes.append([start, 0])
	count = 0

	u_v = [0, 4]
	u_phi = [-math.pi/4.0, math.pi/4.0]

	while (count < nodes):
		#Run till goal is reached
		sample = [start,1]
		n = len(rrt_nodes)
		current_path = []

		while ((checkCollisionSample(obstacle_map, sample) == 1) or (sample[0] in [item[0] for item in rrt_nodes])):
			rand = np.random.random_sample()
			if rand < p_goal:
				sample = [goal, n+1]
			else:
				x_rand = np.random.random_sample()
				y_rand = np.random.random_sample()
				theta_rand = np.random.random_sample()
				sample = [(size_x[0]+x_rand*(size_x[1]-size_x[0]), size_y[0]+y_rand*(size_y[1]-size_y[0]), size_theta[0]+theta_rand*(size_theta[1]-size_theta[0])), n]
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
		new_pt = [(guess[0][0] + delta[0], guess[0][1] + delta[1], guess[0][2]), graph_index]
		#new_pt = [(guess[0][0], guess[0][1], guess[0][2]), graph_index]

		collide, path, controls = findValidSubTraj(obstacle_map,guess,new_pt,u_v,u_phi,size_x,size_y)

		if collide == 0 and len(path) > 0:
			rrt_nodes.append([path[len(path)-1][0], graph_index])
			index = graph_index
			for element in path:
				index += 1
				current_path.append([element[0], index])

			pathList.append(current_path)

			'''print 'sample ' + str(sample)
			print 'new_pt ' + str(new_pt)
			print 'closest ' + str([path[len(path)-1]])
			print 'path ' + str(current_path)
			print 'rrt_nodes ' + str(rrt_nodes)'''

		#check for goal
		if distance(rrt_nodes[len(rrt_nodes)-1], [goal, 0]) < tol:
			return rrt_nodes, pathList, controls

		count = count + 1


	print 'Fail'
	return rrt_nodes, pathList, controls

def findValidSubTraj(obstacle_map,guess,new_pt,u_v,u_phi,size_x,size_y):
	t = np.linspace(0,10,num=20)
	h = 0.5
	m = 50
	d = []
	indeces = []
	u = []
	pathList = []
	for i in range(0,m): #Sample random controls
		path = []
		collision = 0
		found = False
		distc = 1000000
		v_rand = np.random.random_sample()
		phi_rand = np.random.random_sample()
		control_sample = (u_v[0]+v_rand*(u_v[1]-u_v[0]), u_phi[0]+phi_rand*(u_phi[1]-u_phi[0]))

		#Numerically integrate
		[dtheta,t] = rk4theta(guess, control_sample, h, t)
		[dx,t] = rk4x(guess,control_sample, dtheta, h, t)
		[dy,t] = rk4y(guess, control_sample, dtheta, h, t)


		for i in range(0,len(t)-2):
			path.append([(dx[i],dy[i],dtheta[i]),t[i]])

		for i in range(0,len(path)-1):
			if (checkCollisionSample(obstacle_map, path[i]) == 1) or constraintsViolated(path[i][0],size_x,size_y) or checkCollisionPath(obstacle_map,path[i][0],path[i+1][0]):
				collision = 1

		if collision == 0:
			for i in range(0,len(path)):
				dist = distance(path[i], new_pt)
				if dist < distc:
					distc = dist
					node = path[i]
					index = i

			d.append(node)
			indeces.append(index)
			u.append([control_sample,path[index][1]])
			pathList.append(path[0:index])
	distd = 1000000
	if len(pathList) > 0:
		for i in range(0,len(pathList)):
			dist = distance(d[i], new_pt)
			if dist < distd:
				distd = dist
				node = i

		return 0, pathList[node], u[node] 

	else:
		return 1, 0, 0

def constraintsViolated(p1,size_x,size_y):
	if p1[0] > size_x[1] or p1[0] < size_x[0]:
		return 1
	if p1[1] > size_y[1] or p1[1] < size_y[0]:
		return 1
	if np.abs(p1[2]) > 2*math.pi:
		return 1

	return 0

def rk4x(initial, controls, theta, h, t):
	states = []
	states.append(initial[0][0])
	for i in range(2,len(t)-1):
		step = (i-0)*h
		k1 = h*dx(controls[0], theta[i])
		k2 = h*(dx(controls[0], theta[i]) + 0.5*k1)
		k3 = h*(dx(controls[0], theta[i]) + 0.5*k2)
		k4 = h*(dx(controls[0], theta[i]) + k3)
		states.append(states[i-2] + float(1)/float(6)*(k1 + 2*k2 + 2*k3 + k4))

	return states,t

def rk4y(initial, controls, theta, h, t):
	states = []
	states.append(initial[0][1])

	for i in range(2,len(t)-1):
		step = (i-0)*h
		k1 = h*dy(controls[0], theta[i])
		k2 = h*(dy(controls[0], theta[i]) + 0.5*k1)
		k3 = h*(dy(controls[0], theta[i]) + 0.5*k2)
		k4 = h*(dy(controls[0], theta[i]) + k3)
		states.append(states[i-2] + float(1)/float(6)*(k1 + 2*k2 + 2*k3 + k4))

	return states,t

def rk4theta(initial, controls, h, t):
	states = []
	states.append(initial[0][2])

	for i in range(2,len(t)):
		step = (i-0)*h
		k1 = h*dtheta(controls[0],controls[1])
		k2 = h*(dtheta(controls[0], controls[1]) + 0.5*k1)
		k3 = h*(dtheta(controls[0], controls[1]) + 0.5*k2)
		k4 = h*(dtheta(controls[0], controls[1]) + k3)
		states.append(states[i-2] + float(1)/float(6)*(k1 + 2*k2 + 2*k3 + k4))

	return states,t
def dx(u_v,theta):
	return u_v*math.cos(theta)

def dy(u_v,theta):
	return u_v*math.sin(theta)

def dtheta(u_v,phi):
	L = 1
	return u_v*math.tan(phi)

def plotKinematic(obstacle_map, rrt_nodes, pathList, start, goal):
	for item in rrt_nodes:
		plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches

	for i in range(0,len(pathList)):
		path = pathList[i]
		#plt.plot([rrt_nodes[path[0][1]][0][0],path[0][0][0]],[rrt_nodes[path[0][1]][0][1],path[0][0][1]],'r')
		for j in range(0,len(path)-1):
			plt.plot([path[j][0][0],path[j+1][0][0]],[path[j][0][1],path[j+1][0][1]],'r')

	#Plot path

	path = find_pathDyn(rrt_nodes, pathList,start,goal)
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],'g')
	#Make start, end noticable
	plt.plot(start[0], start[1],'g.',markersize=10)
	plt.plot(goal[0], goal[1],'g.',markersize=10)

	plt.show()

def find_pathDyn(rrt_nodes, pathList, start, goal):

	fpath = []
	fpath.append(rrt_nodes[len(rrt_nodes)-1][0])

	for path in pathList:
		if path[len(path)-1][0] == rrt_nodes[len(rrt_nodes)-1][0]:
			for i in range(0,len(path)-1):
				fpath.append(path[len(path)-i-1][0]) 
	pointer = rrt_nodes[len(rrt_nodes)-1][1]

	while (start in [item for item in fpath]) == 0:
		next_node = rrt_nodes[pointer]
		for path in pathList:
			if path[len(path)-1][0] == next_node[0]:
				for i in range(0,len(path)):
					fpath.append(path[len(path)-i-1][0])  
		pointer = next_node[1]
	return fpath

def plotIterDynMDP(obstacle_map, rrt_nodes, pathList, start, itergoal, goal,goalID, title):
	plt.title(title)
	plt.xlabel('X [Units]')
	plt.ylabel('Y [Units]')
	plt.ylim(4097, 0)
	plt.xlim(0, 4097)
	plt.grid()
	for item in rrt_nodes:
		plt.plot(item[0][0],item[0][1],'r.')

	for obstacle in obstacle_map:
		for i in range(0,len(obstacle)):
			if i == len(obstacle)-1:
				plt.plot([obstacle[len(obstacle)-1][0],obstacle[0][0]],[obstacle[len(obstacle)-1][1],obstacle[0][1]],'b')
				break

			plt.plot([obstacle[i][0],obstacle[i+1][0]],[obstacle[i][1],obstacle[i+1][1]],'b')
	#Plot all branches

	for i in range(0,len(pathList)):
		path = pathList[i]
		#plt.plot([rrt_nodes[path[0][1]][0][0],path[0][0][0]],[rrt_nodes[path[0][1]][0][1],path[0][0][1]],'r')
		for j in range(0,len(path)-1):
			plt.plot([path[j][0][0],path[j+1][0][0]],[path[j][0][1],path[j+1][0][1]],'r')

	#Plot path
	path = find_pathDyn(rrt_nodes, pathList,start,itergoal)
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],'g')

	#Make start, end noticable
	plt.plot(start[0], start[1],'g.',markersize=10)
	plt.text(goal[0], goal[1],goalID,fontsize=20)

	plt.show()

def plotOverallDynMDP(obstacle_map, rrt_nodes, path, start, goal,goalID, title):
	plt.title(title)
	plt.xlabel('X [Units]')
	plt.ylabel('Y [Units]')
	plt.ylim(4097, 0)
	plt.xlim(0, 4097)
	plt.grid()
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
	obstacle_map = [O1, O2, O3]
	size_x = (-6,36)
	size_y = (-6,6)
	start = (0,0)
	goal = (35,0)

	#rrt_nodes = rrt_algorithm(obstacle_map,size_x,size_y,5000,0.5,0.05,start,goal,0.25)
	#plot(obstacle_map,rrt_nodes, start, goal)
	size_x = (-6,36)
	size_y = (-6,6)
	size_theta = (0,2*math.pi)
	start = (0,0,0)
	goal = (35,0,0)

	rrt_nodes, pathList = kinematic_rrt(obstacle_map,size_x,size_y,size_theta,50,10,0.05,start,goal,5)
	plotKinematic(obstacle_map,rrt_nodes, pathList, start, goal)
