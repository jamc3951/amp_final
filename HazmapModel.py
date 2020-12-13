#!/usr/bin/python2
#MDP Model based on a processed hazmap
import numpy as np
from ModelSpec import ModelSpec
import sys
from enum import Enum
from Location import *

import matplotlib.pyplot as plt;
import matplotlib.colors as colors



class HazmapModel(ModelSpec):
    def __init__(self, hazImg, goal, *args, **kwds):

        #MDPs on grids are implemented as serial lists - just like image coordinates to an array
        #So the N is the raveled size of the hazmap
        if len(hazImg.shape) != 2:
            print 'HazmapModel: Not a hazmap! Has shape:', hazImg.shape
            return

        print 'Found hazmap with shape:', hazImg.shape
        print 'Setting goal (row, col) to:', goal
        
        self.hazMap = hazImg
        self.N = hazImg.shape[0]*hazImg.shape[1]
        self.height = hazImg.shape[0]
        self.width = hazImg.shape[1]
        self.acts = 9
        self.obs = 9
        
        print 'Using %d states' % self.N
        #Implement a model that's 8-connected:  9-actions (connectivity + stay put)
        super(HazmapModel, self).__init__(N=self.N, acts = self.acts, obs = self.acts)
        self.discount = 0.9
        self.makeTransitions()
        self.goal = goal 
        self.makeRewards()
        #checkReward(self)
        #checkTransition(self)
        
    def makeTransitions(self):
        moveError = 0.01
        moveTrue = 1.0 - (moveError*8)
        #For each action:
        for a in range(0, self.acts):
            #For each state (grid cell): 
            for i in range(0, self.N):
                [y1, x1] = self.getIndex2D(i) #returns as row,col format
                #For each other state (destination)
                for j in range(0, self.N):
                    [y2, x2] = self.getIndex2D(j)

                    #Enumerate possible transitions for each action these two coords represent (if any)
                    if (x1 == x2 and y1==y2):
                        for k in Location:
                            self.px[k.value][i][j] = 0.0
			self.px[Location.Current.value][i][j] =   1.0  #staying put is guaranteed

                    elif(y2 == y1-1 and x2 == x1): #forward
                        for k in Location:
                            self.px[k.value][i][j] = moveError
                        self.px[Location.Forward.value][i][j] =   moveTrue   

                    elif(y2 == y1-1 and x2 == x1+1): #forward-right
			for k in Location:
                            self.px[k.value][i][j] = moveError
		        self.px[Location.FwdRight.value][i][j] =  moveTrue

                    elif(y2 == y1 and x2 == x1 + 1): #pure right
			for k in Location:
                            self.px[k.value][i][j] = moveError
			self.px[Location.Right.value][i][j] =     moveTrue 

		    elif(y2 == y1 + 1 and x2 == x1 + 1): #back right
			for k in Location:
                            self.px[k.value][i][j] = moveError
			self.px[Location.BackRight.value][i][j] = moveTrue 

                    elif(y2 == y1 + 1 and x2 == x1): #back
			for k in Location:
                            self.px[k.value][i][j] = moveError
                        self.px[Location.Back.value][i][j] =      moveTrue 

                    elif(y2 == y1 + 1 and x2 == x1-1): #back left
			for k in Location:
                            self.px[k.value][i][j] = moveError
			self.px[Location.BackLeft.value][i][j] =  moveTrue

                    elif(y2 == y1 and x2 == x1-1): #left
			for k in Location:
                            self.px[k.value][i][j] = moveError
			self.px[Location.Left.value][i][j] =      moveTrue

                    elif(y2 == y1 - 1 and x2 == x1-1): #fwd-left
			for k in Location:
                            self.px[k.value][i][j] = moveError
			self.px[Location.FwdLeft.value][i][j] =   moveTrue

        #Fence off to prevent transitioning off the grid
        #Top:
        ''' indexing issue where fencing is not targeting fences - J and L
        row = 0
        for col in range(0, self.hazMap.shape[1]):
            self.px[Location.Forward.value][row][col] = 0.0
            self.px[Location.FwdLeft.value][row][col] = 0.0
            self.px[Location.FwdRight.value][row][col] = 0.0

        #Bottom
        row = self.hazMap.shape[0]-1
        for col in range(0, self.hazMap.shape[1]):
            self.px[Location.Back.value][row][col] = 0.0
            self.px[Location.BackLeft.value][row][col] = 0.0
            self.px[Location.BackRight.value][row][col] = 0.0

        #Right
        col = self.hazMap.shape[1]-1
        for row in range(0, self.hazMap.shape[0]):
            self.px[Location.Right.value][row][col] = 0.0
            self.px[Location.BackRight.value][row][col] = 0.0
            self.px[Location.FwdRight.value][row][col] = 0.0
            
        #left
        col = 0
        for row in range(0, self.hazMap.shape[0]):
            self.px[Location.Left.value][row][col] = 0.0
            self.px[Location.BackLeft.value][row][col] = 0.0
            self.px[Location.FwdLeft.value][row][col] = 0.0
        '''
        #normalize
        for a in range(0,self.acts):

            #print 'Action ', a, ' table:'
            #print np.reshape(self.px[a], (self.N, self.N))
	    for i in range(0,self.N):
                
		suma = 0; 
		for j in range(0,self.N):
                    suma+=self.px[a][i][j];
                #print 'Row ', i , ' total:', suma
		for j in range(0,self.N):
                    self.px[a][i][j] = self.px[a][i][j]/suma;
                    
    def makeRewards(self):
        #Rewards are assigned to not being on obstacles and reaching the goal
        #in the hazmap, black (0) is traversable and white(255) is an obstacle
        self.obstacleReward = -100#-960#-150   #+101 from OG Steve on all +10 on obstacle
        self.stationaryReward = -2#-90#150#100
        self.goalReward = 200 #250#201

        
        #Base penalty for moving to free space
        for j in range(0, len(Location)):
            for i in range(0, self.N):
                self.R[j][i] = self.stationaryReward
                self.R_values[i] = self.stationaryReward
        
        #Handle the borders first to put obstacles around the edge of the map - fence the robot in
        #Row 0:
        for col in range(0, self.hazMap.shape[1]-1):
            #self.R[Location.Forward.value][self.getIndex(0,col)] = self.obstacleReward
            #self.R[Location.FwdLeft.value][self.getIndex(0,col)] = self.obstacleReward
            #self.R[Location.FwdRight.value][self.getIndex(0,col)] = self.obstacleReward
            
            if self.hazMap[0][col] > 0:
                self.R_values[self.getIndex(0,col)] = self.obstacleReward
                self.R[Location.Current.value][self.getIndex(0,col)] = self.obstacleReward
                self.R[Location.Forward.value][self.getIndex(1,col)] = self.obstacleReward
                
                if col < (self.hazMap.shape[1]-1):
                    self.R[Location.Right.value][self.getIndex(0, col-1)] = self.obstacleReward
                    self.R[Location.FwdRight.value][self.getIndex(1,col-1)] = self.obstacleReward
                if col > 0:
                    self.R[Location.Left.value][self.getIndex(0, col+1)] = self.obstacleReward
                    self.R[Location.FwdLeft.value][self.getIndex(1,col+1)] = self.obstacleReward

        #Row end:
        maxRow = self.hazMap.shape[0] - 1
        for col in range(0, self.hazMap.shape[1]-1):
            #self.R[Location.Back.value][self.getIndex(maxRow,col)] = self.obstacleReward
            #self.R[Location.BackLeft.value][self.getIndex(maxRow,col)] = self.obstacleReward
            #self.R[Location.BackRight.value][self.getIndex(maxRow,col)] = self.obstacleReward
            
            if self.hazMap[-1][col] > 0:
                self.R_values[self.getIndex(maxRow,col)] = self.obstacleReward
                self.R[Location.Current.value][self.getIndex(maxRow,col)] = self.obstacleReward
                self.R[Location.Back.value][self.getIndex(maxRow-2,col)] = self.obstacleReward
                if col < (self.hazMap.shape[1]-1):
                    self.R[Location.Right.value][self.getIndex(maxRow-1, col-1)] = self.obstacleReward
                    self.R[Location.BackRight.value][self.getIndex(maxRow-2,col-1)] = self.obstacleReward
                if col > 0:
                    self.R[Location.Left.value][self.getIndex(maxRow-1, col+1)] = self.obstacleReward
                    self.R[Location.BackLeft.value][self.getIndex(maxRow-2,col+1)] = self.obstacleReward

        #Left col:
        for row in range(0, self.hazMap.shape[0]-1):
            #self.R[Location.FwdLeft.value][self.getIndex(row,0)] = self.obstacleReward
            #self.R[Location.BackLeft.value][self.getIndex(row,0)] = self.obstacleReward
            #self.R[Location.Left.value][self.getIndex(row,0)] = self.obstacleReward
            
            if self.hazMap[row][0] > 0:
                self.R_values[self.getIndex(row,0)] = self.obstacleReward
                self.R[Location.Current.value][self.getIndex(row,0)] = self.obstacleReward
                self.R[Location.Forward.value][self.getIndex(row+1,0)] = self.obstacleReward
                self.R[Location.Back.value][self.getIndex(row-1,0)] = self.obstacleReward
                self.R[Location.Left.value][self.getIndex(row,1)] = self.obstacleReward
                self.R[Location.FwdLeft.value][self.getIndex(row+1,1)] = self.obstacleReward
                self.R[Location.BackLeft.value][self.getIndex(row-1,1)] = self.obstacleReward
                
        #Right col:
        maxCol = self.hazMap.shape[1] - 1
        for row in range(0, self.hazMap.shape[0]-1):
            #self.R[Location.FwdRight.value][self.getIndex(row,maxCol)] = self.obstacleReward
            #self.R[Location.BackRight.value][self.getIndex(row,maxCol)] = self.obstacleReward
            #self.R[Location.Right.value][self.getIndex(row,maxCol)] = self.obstacleReward
            
            if self.hazMap[row][maxCol] > 0:
                self.R_values[self.getIndex(row,maxCol)] = self.obstacleReward
                self.R[Location.Current.value][self.getIndex(row,maxCol)] = self.obstacleReward
                self.R[Location.Forward.value][self.getIndex(row+1,maxCol)] = self.obstacleReward
                self.R[Location.Back.value][self.getIndex(row-1,maxCol)] = self.obstacleReward
                
                self.R[Location.Right.value][self.getIndex(row,maxCol-1)] = self.obstacleReward
                self.R[Location.FwdRight.value][self.getIndex(row+1,maxCol-1)] = self.obstacleReward
                self.R[Location.BackRight.value][self.getIndex(row-1,maxCol-1)] = self.obstacleReward


        #Handle the rest of the map generically:
        for row in range(1, self.hazMap.shape[0]-1):
            for col in range(1, self.hazMap.shape[1]-1):
               
                #Check each direction for an obstacle
                #Action consequences
                if self.hazMap[row][col] > 0:
                    self.R_values[self.getIndex(row,col)] = self.obstacleReward
                    self.R[Location.Current.value][self.getIndex(row,col)] = self.obstacleReward
                    self.R[Location.Left.value][self.getIndex(row,col+1)] = self.obstacleReward
                    self.R[Location.Forward.value][self.getIndex(row+1,col)] = self.obstacleReward
                    self.R[Location.Back.value][self.getIndex(row-1,col)] = self.obstacleReward
                    self.R[Location.Right.value][self.getIndex(row,col-1)] = self.obstacleReward
                    self.R[Location.FwdLeft.value][self.getIndex(row+1,col+1)] = self.obstacleReward
                    self.R[Location.FwdRight.value][self.getIndex(row+1,col-1)] = self.obstacleReward
                    self.R[Location.BackRight.value][self.getIndex(row-1,col-1)] = self.obstacleReward
                    self.R[Location.BackLeft.value][self.getIndex(row-1,col+1)] = self.obstacleReward
                
        
        #Draw the goal in each action map..., assuming goals are specified in row,col format
        #Assumes that the goals aren't on the borders...
        
        print 'goal (row, col):', self.goal
        self.R_values[self.getIndex(self.goal[0],self.goal[1])] = self.goalReward
        self.R[Location.Current.value][self.getIndex(self.goal[0],self.goal[1])] = self.goalReward
        self.R[Location.Back.value][self.getIndex(self.goal[0]-1,self.goal[1])] = self.goalReward
        self.R[Location.BackRight.value][self.getIndex(self.goal[0]-1,self.goal[1]-1)] = self.goalReward
        self.R[Location.Right.value][self.getIndex(self.goal[0],self.goal[1]-1)] = self.goalReward
        self.R[Location.FwdRight.value][self.getIndex(self.goal[0]+1,self.goal[1]-1)] = self.goalReward
        self.R[Location.Forward.value][self.getIndex(self.goal[0]+1,self.goal[1])] = self.goalReward
        self.R[Location.FwdLeft.value][self.getIndex(self.goal[0]+1,self.goal[1]+1)] = self.goalReward
        self.R[Location.Left.value][self.getIndex(self.goal[0],self.goal[1]+1)] = self.goalReward
        self.R[Location.BackLeft.value][self.getIndex(self.goal[0]-1,self.goal[1]+1)] = self.goalReward

        
    def getIndex(self, row, col):
        return row*self.width+col
    
    def getIndex2D(self, n):
        #Return as row, col
        return n // self.width, n % self.width
    
    def convertToGrid(self,a):
	b = np.zeros(shape=(self.N,self.N),dtype=np.float16);  
        for i in range(0,self.N):
            b[i//self.height][i%self.width]=a[i]; 
	return b;
    
    def getAction(self, act):
        #Return the enum that matches the integer
        return Location(int(act))
    
def checkReward(m):
	'''
	x = [i for i in range(0,m.N)]; 
	plt.plot(x,m.R[0],c='r');
	plt.plot(x,m.R[1],c='b'); 
	plt.plot(x,m.R[2],c='g');  
	plt.legend(['Left','Right','Stay']); 
	plt.show()
	'''
       
        plots = []
        nRows = int(np.sqrt(len(Location)))
        nCols = nRows
	fig,axarr = plt.subplots(nRows, nCols);
        fig.suptitle("Rewards", fontsize=14)
        for i in range(0, nRows):
            for j in range(0, nCols):
                #print np.reshape(m.R[i*nCols+j], (m.height, m.width)).dtype
                plot = axarr[i][j].imshow(np.reshape(m.R[i*nCols+j], (m.height, m.width)).astype(np.float32),cmap = 'binary',norm=colors.Normalize(vmin=np.min(m.R), vmax=np.max(m.R)), alpha=0.5)
                #plot = axarr[i][j].contourf(np.reshape(m.R[i*nCols+j], (m.height, m.width)).astype(np.float32),cmap = 'binary', alpha=0.5)
                #axarr[i][j].imshow(m.hazMap, cmap='binary', alpha=0.5)
                axarr[i][j].set_ylim(m.height-0.5, -0.5)
                axarr[i][j].set_title('Action %d:%s' % (i*nCols+j, str(Location(i*nCols+j))))
                plots.append(plot)
                
        
        
        plt.show() 
       
        '''
        fig,axarr = plt.subplots(1);
        plot = axarr.contourf(np.reshape(m.R[0], (m.height, m.width)),cmap = 'plasma')
        fig.colorbar(plot)
        plt.show()
        '''

def checkTransition(m):
    
        '''
        transitions = np.zeros((len(Location), m.N, m.N))
        
	x0 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x1 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x2 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x3 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x4 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
        x5 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x6 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x7 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
	x8 = [[0 for i in range(0,m.N)] for j in range(0,m.N)]; 
        '''
        
	x, y = np.mgrid[0:m.N:1, 0:m.N:1]

        '''
	for i in range(0,m.N):
		for j in range(0,m.N):
			x0[i][j] = m.px[0][i][j]; 
			x1[i][j] = m.px[1][i][j]; 
			x2[i][j] = m.px[2][i][j];
			x3[i][j] = m.px[3][i][j]; 
			x4[i][j] = m.px[4][i][j];
                        x5[i][j] = m.px[5][i][j]; 
			x6[i][j] = m.px[6][i][j]; 
			x7[i][j] = m.px[7][i][j];
			x8[i][j] = m.px[8][i][j]; 
        '''


        nRows = int(np.sqrt(len(Location)))
        nCols = nRows
	fig,axarr = plt.subplots(nRows, nCols);
        plots = []
        for i in range(0, nRows):
            for j in range(0, nCols):
                plot = axarr[i][j].contourf(x,y,m.px[i*nCols+j])
                axarr[i][j].set_ylim(max( axarr[i][j].get_ylim()), min( axarr[i][j].get_ylim()))
                axarr[i][j].set_title('Action %d:%s' % (i*nCols+j, str(Location(i*nCols+j))))
                plots.append(plot)
        '''
                
	axarr[0].contourf(x,y,x0); 
	axarr[1].contourf(x,y,x1); 
	axarr[2].contourf(x,y,x2); 
	axarr[3].contourf(x,y,x3); 
	axarr[4].contourf(x,y,x4);
        axarr[5].contourf(x,y,x5); 
	axarr[6].contourf(x,y,x6); 
	axarr[7].contourf(x,y,x7); 
	axarr[8].contourf(x,y,x8); 
        '''
	plt.show();
        
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'Usage: %s <hazard map.npy>' % sys.argv[0]
    hazMap = np.load(sys.argv[1])
    
    m = HazmapModel(hazMap, (1,1)); 
    print '5:', m.getIndex2D(5)
    print '(2,2): ', m.getIndex(2,2)
    
    #raw_input()
    #checkReward(m);
    #checkObs(m); 
    checkTransition(m); 
    
