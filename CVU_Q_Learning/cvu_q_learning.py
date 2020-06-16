import math
import random
import sys
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

# State definitions
LOST = 3
LEFT = 0
CENTER  = 1
RIGHT = 2

# Actions
TURN_LEFT = 0
FORWARD = 1
TURN_RIGHT = 2

# Rewards
REWARD = 1.0
PENALTY = -1.0
LOW_PENALTY = -0.05

RAND_LIMIT = 10000


M1 = 0
M2 = 1

MIN = 0
MAX = 1


# QLearning Class
# Description: Implements all necessary estructure for Q-learning algorithm iterations
# Created by Mateus Franco @ 01/10/2018
# Change History:
class Q_Learning :

	# Method Name: __init__ (Constructor)
	# Description: Instantiate a QLearning object with all minimum parameters
	# Created by Mateus Franco @ 01/10/2018
	# Change History:
	def __init__(self,name,MCU,imageWidth,maxSpeed,ALPHA,LAMBDA,states,actions,rewards,camera) :
		self.numberOfActions = len(actions)
		self.numberOfStates = len(states)
		self.MCU = MCU
		self.P = [0.0,0.0,0.0]	
		self.Q = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
		self.rewards = [0.0,0.0,0.0,0.0]
		self.currentState = 0
		self.nextState = 0
		self.action = 0
		self.ALPHA = ALPHA
		self.LAMBDA = LAMBDA
		self.states = states
		self.currentAction = 0
		self.actions = actions
		self.rewards = rewards
		self.name = name
		self.initQ()
		print 'New Q_Learning defined: '+self.name+'\n'

	# Method Name: initQ
	# Description: Iniialize Q table with zeros
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def initQ(self) :
		for i in range(self.numberOfStates) :

			for j in range(self.numberOfActions) :
				self.Q[i][j] = 0.0

	# Method Name: printQTable
	# Description: Formated print method for Q table showing iteraction number on header
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def printQTable(self, iteraction) :
		print "===================================="
		print "============ Q Table ==============="
		print "========================== it "+str(iteraction)
		sys.stdout.flush()
		for i in range(self.numberOfStates) :
			for j in range(self.numberOfActions) :
				sys.stdout.write(str(round(self.Q[i][j])))
				sys.stdout.write('\t')
			print ''

	# Method Name: takeAction
	# Description: Apply and choosen action sending a message to MCU
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def takeAction(self,action) :
		global M1
		global M2
		print action
		print 'Send to MCU: '+str(self.actions[action][M1]) + ',' + str(self.actions[action][M2])

	# Method Name: max_a_Q
	# Description: Find the maximum Q value for an Action in a given State
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def max_a_Q(self, state) :
		max_a_Q = self.Q[state][0]
		for i in range(self.numberOfActions) :
			max_a_Q = max(max_a_Q, self.Q[state][i])
		return max_a_Q

	# Method Name: selectAction
	# Description: Choose and Action base on cumulative probability array
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def selectAction(self,state) :
		global RAND_LIMIT
		Sum = 0
		P = [0.0,0.0,0.0]
		max_s_Q = self.Q[state][0]
		min_s_Q = self.Q[state][0]

		for i in range(self.numberOfActions) :
			if(self.Q[state][i] > max_s_Q) : max_s_Q = self.Q[state][i]
			if(self.Q[state][i] < min_s_Q) : min_s_Q = self.Q[state][i]

		if (max_s_Q - min_s_Q > 30) : min_s_Q = max_s_Q - 30
		
		print 'min: '+str(min_s_Q)
		print 'max: '+str(max_s_Q)

		for i in range(self.numberOfActions) :
			self.P[i] = math.exp(self.Q[state][i] - min_s_Q)
			Sum = Sum + self.P[i]
			
			print "P["+str(i)+"] = "+str(self.P[i])+" - Sum = "+str(Sum)
		
		inv_sum = 1.0/Sum
		accum = 0
		rand = random.randrange(RAND_LIMIT)
		action = self.numberOfActions - 1

		print 'Rand selected: '+str(rand)
		
		for i in range(self.numberOfActions) :
			self.P[i] = self.P[i] * inv_sum
			accum = accum + self.P[i]
			
			print "P["+str(i)+"] = "+str(self.P[i])+" - Accum = "+str(accum)
			
			print accum*RAND_LIMIT

			if(rand <= accum*RAND_LIMIT) :
				action = i
				break

		return action

	# Method Name: getState
	# Description: Compute the State based on line center position, obtained from CVU
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def getState(self,centerPosition) :
		global LEFT
		global CENTER
		global RIGHT
		global LOST
		global MIN
		global MAX
		for s in range (len(states) - 2) :
			if centerPosition >= states[s][MIN] and centerPosition <= states[s][MAX] :
				return s
		return len(states) - 1

	# Method Name: start
	# Description: Simple method to put the robot to start movement
	# Created by Mateus Franco @ 02/10/2018
	# Change History:		
	def start(self) :
		self.takeAction(FORWARD)

	# Method Name: learn
	# Description: 	Implements QLearning iteractions. In even steps, get the current State and select the Action to take.
	# 				In odd steps, get the nextState and updates Q table
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def learn(self, lineCenter, step) :
		if(step == 0) :
			self.currentState = self.getState(lineCenter)
			self.currentAction = self.selectAction(self.currentState)
			self.takeAction(self.currentAction)
		else :
			self.nextState = self.getState(lineCenter)
			self.Q[self.currentState][self.currentAction] = ((1 - self.ALPHA) * self.Q[self.currentState][self.currentAction] +self.ALPHA * (self.rewards[self.nextState] + self.LAMBDA * self.max_a_Q(self.nextState)))
			self.currentState = self.nextState

# AdaptativeLineDetector Class
# Description: 	Implements all structure necessary to obtain line position and operates QLearning object
#				in the learning process
# Created by Mateus Franco @ 03/10/2018
# Change History:
class AdaptativeLineDetector :

	# Method Name: __init__ (Constructor)
	# Description: 	Instantiate a QLearning object with all minimum parameters
	# Created by Mateus Franco @ 03/10/2018
	# Change History:
	def __init__(self,width,height,frameRate,windowWidth,windowHeight,agent) :
		self.height = height
		self.width = width
		self.windowHeight = windowHeight
		self.windowWidth = windowWidth
		self.camera = PiCamera()
		self.camera.resolution = (self.width,self.height)
		self.camera.framerate = frameRate
		self.rawCapture = PiRGBArray(self.camera,size=(self.width,self.height))
		time.sleep(0.1)
		self.agent = agent
		print 'New AdaptativeLineDetector defined: '+str(self.camera)+' with agent '+self.agent.name+'\n'

	# Method Name: 	processNewFrame
	# Description:	Get a new frame from camera, crop the frame, apply filters and return the white line contour	
	# Created by Mateus Franco @ 03/10/2018
	# Change History:
	def processNewFrame(self,frame) :
		image = frame.array
		crop_img = image[0:self.windowHeight, 0:self.windowWidth]
		gray = cv2.cvtColor(crop_img,cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray,(5,5),0)
		ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY)
		_,contours,hierarchy = cv2.findContours(thresh.copy(),1,cv2.CHAIN_APPROX_NONE)
		return contours

	# Method Name: 	getLineCenter
	# Description:	Get x coordinate from white line center in the frame returning its position.
	#				This method return 0 if the line is not found in the frame (Lost State)	
	# Created by Mateus Franco @ 03/10/2018
	# Change History:
	def getLineCenter(self,contours) :

		if len(contours) > 0 :
			c = max(contours, key=cv2.contourArea)
			M = cv2.moments(c)
			if(M['m00'] != 0) :
				cx = int(M['m10']/M['m00'])
			return cx
		else :
			return 0

	# Method Name: 	learnHowToFollow
	# Description:	Obtain a new frame in each iteraction, define QLearning step and calls QLearning methods
	#				to update states, actions and Q table
	# Created by Mateus Franco @ 03/10/2018
	# Change History:
	def learnHowToFollow(self) :
		i = 0
		j = 0
		for frame in self.camera.capture_continuous(self.rawCapture,format='bgr',use_video_port=True) :
			if(i == 0) :
				lineCenter =  int(self.getLineCenter(self.processNewFrame(frame)))
				self.agent.learn(lineCenter,i)
				i = 1
			else :
				lineCenter =  int(self.getLineCenter(self.processNewFrame(frame)))
				self.agent.learn(lineCenter,i)
				i = 0

			self.rawCapture.truncate(0)
			j = j + 1
			if(j > 100) : break
		self.agent.printQTable(j)

	#def buildStateAndActionsMapping(self) :
	#def followAsLearned(self) :

# main

rewards = [0,0,0,0]

rewards[CENTER] = REWARD
rewards[LEFT] = LOW_PENALTY
rewards[RIGHT] = LOW_PENALTY
rewards[LOST] = PENALTY

states = [[1,80],[81,160],[161,240],[0,0]]
actions = [[0,100],[100,100],[100,0]]

MCU = 1
CAM = 1

agent = Q_Learning('Vision Q-learning',MCU,240,100,0.1,0.9,states,actions,rewards,CAM)

lineDetector = AdaptativeLineDetector(240,60,60,240,60,agent)
lineDetector.learnHowToFollow()
#agent.learn()

#agent.printQTable(2000)



