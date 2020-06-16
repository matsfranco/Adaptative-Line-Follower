import math
import random
import sys
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import serial
from simple_salesforce import Salesforce

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
class QLearning :

	# Method Name: __init__ (Constructor)
	# Description: Instantiate a QLearning object with all minimum parameters
	# Created by Mateus Franco @ 01/10/2018
	# Change History:
	#	1. Mateus Franco @ 20/10/2018
	#		dataLogger object added to this class, currentLinePosition and nextLinePosition 
	def __init__(self,name,MCU,imageWidth,maxSpeed,ALPHA,LAMBDA,states,actions,rewards,camer,actionDelay,dataLogger) :
		self.learningPhase = True
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
		self.message = ''
		self.actionDelay = actionDelay
		self.dataLogger = dataLogger
		salesforce.insertLearningProcess('ALF Robot')
		self.initQ()
		print 'New QLearning defined: '+self.name+'\n'

	# Method Name: initQ
	# Description: Iniialize Q table with zeros
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def initQ(self) :
		for i in range(self.numberOfStates) :
			for j in range(self.numberOfActions) :
				self.Q[i][j] = 0.0

	# Method Name: loadQ
	# Description: Iniialize Q table with another Q table
	# Created by Mateus Franco @ 21/10/2018
	# Change History:
	def loadQ(self, otherQ) :
		for i in range(self.numberOfStates) :
			for j in range(self.numberOfActions) :
				self.Q[i][j] = otherQ[i][j]

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
				sys.stdout.write(str(round(self.Q[i][j],4)))
				sys.stdout.write('\t')
			print ''

	# Method Name: takeAction
	# Description: Apply and choosen action sending a message to MCU
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def takeAction(self,action) :
		global M1
		global M2
		print "Actions["+str(action)+"] selected - "+str(self.actions[action])
		self.MCU.setSpeeds(self.actions[action][M1],self.actions[action][M2])
		print "QLearning.takeAction() - Action taken = "+str(action)
		#if(self.learningPhase == True) : time.sleep(self.actionDelay)

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
		
		#print 'min: '+str(min_s_Q)
		#print 'max: '+str(max_s_Q)

		for i in range(self.numberOfActions) :
			self.P[i] = math.exp(self.Q[state][i] - min_s_Q)
			Sum = Sum + self.P[i]
			
			#print "P["+str(i)+"] = "+str(self.P[i])+" - Sum = "+str(Sum)
		
		inv_sum = 1.0/Sum
		accum = 0
		rand = random.randrange(RAND_LIMIT)
		action = self.numberOfActions - 1

		#print 'Rand selected: '+str(rand)
		
		for i in range(self.numberOfActions) :
			self.P[i] = self.P[i] * inv_sum
			accum = accum + self.P[i]
			
			#print "P["+str(i)+"] = "+str(self.P[i])+" - Accum = "+str(accum)
			
			#print accum*RAND_LIMIT

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
		for s in range (len(states) - 1) :
			if centerPosition >= states[s][MIN] and centerPosition <= states[s][MAX] :
				return s
		return len(states) - 1

	# Method Name: start
	# Description: Simple method to put the robot to start movement
	# Created by Mateus Franco @ 02/10/2018
	# Change History:		
	def start(self) :
		self.takeAction(FORWARD)
		return 1

	# Method Name: learn
	# Description: 	Implements QLearning iteractions. In even steps, get the current State and select the Action to take.
	# 				In odd steps, get the nextState and updates Q table
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def learn(self, lineCenter, step,iteration) :
		if(step == 0) :
			self.currentLinePosition = lineCenter 
			self.currentState = self.getState(self.currentLinePosition)
			self.currentAction = self.selectAction(self.currentState)
			print "QLearning.learn(): Line Center = "+str(lineCenter)+"\tCurrent State = "+str(self.currentState)
			self.takeAction(self.currentAction)
		else :
			self.nextLinePosition = lineCenter
			self.nextState = self.getState(self.nextLinePosition)
			self.Q[self.currentState][self.currentAction] = ((1.0 - self.ALPHA) * self.Q[self.currentState][self.currentAction] +self.ALPHA * (self.rewards[self.nextState] + self.LAMBDA * self.max_a_Q(self.nextState)))
			self.MCU.stopMotors()
			self.dataLogger.sendToSalesforce(iteration,self.currentState,self.currentAction,self.nextState,self.Q,self.rewards[self.nextState],self.currentLinePosition,self.nextLinePosition)
			self.currentState = self.nextState
			print "QLearning.learn(): Line Center = "+str(lineCenter)+"\tNext State = "+str(self.currentState)


# MCU Class
# Description: 	Implements all structure necessary connect, send and receive informations from the 
#				Motor Control Unit (MCU) attached to a specific serial connection
# Created by Mateus Franco @ 06/10/2018
# Change History:
class MCU :

	# Method Name: __init__ (Constructor)
	# Description: Instantiate a MCU object with all minimum parameters
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	#	07/10/2018 - Mateus Franco
	#		Six pid gains parameters replaced by two arrays of parameters: pid1Gains and pid2Gains
	def __init__(self,name,port,baudRate,pid1Gains,pid2Gains) :
		self.connectionName = name
		self.port = port
		self.baudRate = baudRate
		self.connectionStatus = 0
		self.m1Speed = 0
		self.m2Speed = 0
		self.kp1 = pid1Gains[0]
		self.ki1 = pid1Gains[1]
		self.kd1 = pid1Gains[2]
		self.kp2 = pid2Gains[0]
		self.ki2 = pid2Gains[1]
		self.kd2 = pid2Gains[2]
		self.lastCommand = ""
		print "New Serial Device: "+name+ " created as MCU @ "+str(port)+ " / "+str(baudRate)+ "bps\n"
	
	# Method Name: recvMessage 
	# Description: Receive a string from MCU and return this message as a string without \r\n in the end
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def recvMessage(self) :
		message = self.port.readline()
		message = message.replace('\r\n','')
		print "Received from "+self.connectionName+": "+str(message)+"\n"
		return message
	
	# Method Name: connect
	# Description: 	Create a serial port and estabilshes the serial connection between CVU and MCU apply a
	#				handshade kind protocol. After send the connection message waits an echo as response.
	#				If resonse don't come, another attemp is made periodically
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def connect(self) :
		print "Connecting to MCU...\n"
		self.port = serial.Serial(self.port,self.baudRate)
		time.sleep(2)
		self.sendMessage("3000\n")
		time.sleep(0.1)
		while (int(self.recvMessage()) != 3000) : 
			print "Failed to connect to MCU. Trying again!\n"
		print "MCU connected!\n"
		connectionStatus  = 1
		time.sleep(1)

	# Method Name: connect
	# Description: 	Send a string through MCU serial connection
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	#	07/10/2018 - Mateus Franco
	#		Improvements to prevent sequencial multiple message sending
	def sendMessage(self,message) :
		# message 3003 is an exception. It needs to be sent two times to prevent overshoots in step response tests
		if(message != self.lastCommand or message == '3003\n') :
			self.lastCommand = message
			self.port.write(message)
			print "MCU.sendMessage: sent to "+self.connectionName+" - "+message
		else :
			print "MCU.sendMessage: Command already sent\n"
	# Method Name: 	setSpeeds
	# Description: 	Send a string with M1 and M2 angular speeds in rpm. Return 1 after message sent.
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def setSpeeds(self, m1Speed, m2Speed) :
		self.m1Speed = m1Speed
		self.m2Speed = m2Speed
		self.sendMessage("1000,"+str(m1Speed)+","+str(m2Speed)+"\n")
		return 1

	# Method Name: 	stopMotors
	# Description: 	Send a string with instruction to stop motor movement and renew controller parameters.
	#				This method returns 1 after message is sent.
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def stopMotors(self) :
		# SPECIAL NOTE:
		# 3003 instruction is sent two times: the first to stop motors and the second to renew controller
		# internal values. It's necessary to get a correct step response from 0 rpm without overshoots
		self.sendMessage("3003\n");
		self.sendMessage("3003\n");
		return 1

	# Method Name: 	setControllerGauns
	# Description: 	Send a string to update PID controller gains basend on its ID.
	#				This method returns the corresponding controllerId updated after message is sent.
	#				returning -1 if no controller is found.
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def setControllerGains(self, controllerId, kp, ki, kd) :
		# controllerId = 1 refers to M1 PID Controller
		if(controllerId == 1) : 
			self.kp1 = kp
			self.ki1 = ki
			self.kd1 = kd
			self.sendMessage("3001,"+str(kp)+","+str(ki)+","+str(kd)+"\n")
			return 1
		# controllerId = 2 refers to M2 PID Controller
		else :
			if(controllerId == 2) :
				self.kp2 = kp
				self.ki2 = ki
				self.kd2 = kd
				self.sendMessage("3002,"+str(kp)+","+str(ki)+","+str(kd)+"\n")
				return 2
			else :
				return -1

	# Method Name: 	enableDataStreaming
	# Description: 	Enables data streaming from MCU. This streaming contains a csv string with \r\n ending
	#				This method returns 1 to indicate that Streaming is ON (1)
	# Created by Mateus Franco @ 06/10/2018
	# Change History:			
	def enableDataStreaming(self) :
		self.sendMessage("4001\n")
		return 1
	
	# Method Name: 	disableDataStreming
	# Description: 	Disables data streaming from MCU.
	#				This method returns 0 to indicate that Streaming is OFF (0)
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def disableDataStreming(self) :
		self.sendMessage("4002\n")
		return 0

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
		self.stateActionMap = []
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
		ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_BINARY)
		_,contours,hierarchy = cv2.findContours(thresh.copy(),1,cv2.CHAIN_APPROX_NONE)
		#cv2.imshow('Crop',crop_img)
		#cv2.imshow('gray',gray)
		#cv2.imshow('ret',thresh)
		#key = cv2.waitKey(1) & 0xFF
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
	#	08/10/2018 - Mateus Franco
	#		raw_input call to stop running to easy practical test routine
	def learnHowToFollow(self,iterationLimit) :
		step = 0
		iteration = 1
		for frame in self.camera.capture_continuous(self.rawCapture,format='bgr',use_video_port=True) :
			print "\n---------------------------------------------------"
			if(step == 0) :
				print "---------- CURRENT ----------"
				lineCenter = self.getLineCenter(self.processNewFrame(frame))
				self.agent.learn(lineCenter,step,iteration)
				step = 1
			else :
				print "---------- NEXT ----------"
				lineCenter = self.getLineCenter(self.processNewFrame(frame))
				self.agent.learn(lineCenter,step,iteration)
				step = 0
				self.agent.printQTable(iteration)

			self.rawCapture.truncate(0)
			iteration = iteration + 0.5
			if(iteration > iterationLimit) : break
		self.agent.learningPhase = False
		self.agent.printQTable(iteraction)

	# Method Name: 	getLinePosition
	# Description:	Obtain a new frame in each iteraction, calculate line center position and
	#				gets the corresponding state. Great for debugging.
	# Created by Mateus Franco @ 10/10/2018
	# Change History:
	def getLinePosition(self) :
		for frame in self.camera.capture_continuous(self.rawCapture,format='bgr',use_video_port=True) :
			lineCenter = self.getLineCenter(self.processNewFrame(frame))
			print str(lineCenter) +", "+str(self.agent.getState(lineCenter))
			self.rawCapture.truncate(0)

	# Method Name: 	getMaxIndex
	# Description:	Obtain the index of the maximum value of given array.
	# Created by Mateus Franco @ 10/10/2018
	# Change History:
	def getMaxIndex(self,probList) :
		i = 0
		maxProb = -1
		maxIndex = 0
		while i < len(self.agent.actions) :
			if probList[i] > maxProb : 
				maxProb = probList[i]
				maxIndex = i
			i = i + 1
		return maxIndex

	# Method Name: 	consolidateLearning
	# Description:	Creates an array with direct mapping between state and actions
	# Created by Mateus Franco @ 10/10/2018
	# Change History:
	def consolidateLearning(self) :
		self.stateActionMap = []
		for state in range(len(self.agent.states)) :
			maxProbIndex = self.getMaxIndex(self.agent.Q[state])
			self.stateActionMap.append(maxProbIndex)
		print "consolidateLearning(): Final State - Action Map"
		for state in range(len(self.agent.states)) :
			print "["+str(state)+"] - "+str(self.stateActionMap[state])
		
	# Method Name: 	runAsLearned
	# Description:	Run over the line following the state-action map builded after learning phase
	# Created by Mateus Franco @ 12/10/2018
	# Change History:
	def runAsLearned(self) :
		for frame in self.camera.capture_continuous(self.rawCapture,format='bgr',use_video_port=True) :
			lineCenter = self.getLineCenter(self.processNewFrame(frame))
			print str(lineCenter) +", "+str(self.agent.getState(lineCenter))
			self.agent.takeAction(self.stateActionMap[self.agent.getState(lineCenter)])
			time.sleep(0.05)
			self.rawCapture.truncate(0)

class LogDataSet : 

	def __init__(self,iterationNumber,currenState,actionTaken,nextState,QTable,rewards,currentLinePosition,nextLinePosition) :
		self.iterationNumber = 0
		self.currenState = 0
		self.actionTaken = 0
		self.nextState = 0
		self.QTable = []
		self.rewards = 0.0
		self.currentLinePosition = 0
		self.nextLinePosition = 0


class DataLogger :

	def __init__(self, salesforceIntegrator) :
		self.salesforce = salesforceIntegrator
		self.log = []

	def store(self,iterationNumber,currentState,actionTaken,nextState,qTable,reward,currentLinePosition,nextLinePosition) :
		self.newLogDataSet = LogDataSet(iterationNumber,currenState,actionTaken,nextState,qTable,reward,currentLinePosition,nextLinePosition)
		self.log.append(self.newLogDataSet)
		return 1

	def sendToSalesforce(self,iterationNumber,currentState,actionTaken,nextState,qTable,reward,currentLinePosition,nextLinePosition) :
		salesforce.insertIteration(iterationNumber,currentState,actionTaken,nextState,reward,currentLinePosition,nextLinePosition)
		salesforce.insertQMatrix(qTable,4,3)
		return 1

class QMatrixCell : 

	def __init__(self,iterationId,row,column,value) :
		self.Iteration_Number__c = iterationId
		self.Row__c = row
		self.Column__c = column
		self.Value__c = value
		self.Name = "Cell "+str(row)+","+str(column)

	def toDictionary(self) :
		data = {}
		data['Name'] = self.Name
		data['Iteration_Number__c'] = self.Iteration_Number__c
		data['Row__c'] = self.Row__c
		data['Column__c'] = self.Column__c
		data['Value__c'] = self.Value__c
		return data

	def __repr__(self):
		return json.dumps(self, default=jsonDefault)

class SalesforceIntegration :

	def __init__(self,username,password,organizationId,security_token) :
		self.username = username 
		self.password = password
		self.organizationId = organizationId 
		self.security_token = security_token 

	def startSession(self) :
		self.salesforce = Salesforce(username=self.username,
				password=self.password,
				organizationId=self.organizationId,
				security_token=self.security_token)
		print "Connected to Salesforce!"

	def insertLearningProcess(self,deviceName) :
		response =  str(self.salesforce.Learning_Process__c.create({'Device_Name__c':deviceName}))
		self.LearningProcessId = response[23:41]
		return self.LearningProcessId

	def insertIteration(self,iterationNumber,currentState,actionTaken,nextState,rewardSignal,currsentLinePos,nextLinePos) :
		response =  str(self.salesforce.Iteration__c.create(
			{	'Name':iterationNumber,
				'Learning_Process__c':self.LearningProcessId,
				'Current_State__c':currentState,
				'Action_Taken__c':actionTaken,
				'Next_State__c':nextState,
				'Reward_Signal__c':rewardSignal,
				'Current_Line_Position__c':currsentLinePos,
				'Next_Line_Position__c':nextLinePos 	}))
		self.currentIterationId = response[23:41]
		return self.currentIterationId 

	def insertQMatrixCell(self,row,column,value) :
		response = str(self.salesforce.Q_Matrix_Cell__c.create(
			{	'Name':"Cell "+str(row)+","+str(column),
				'Iteration_Number__c':self.currentIterationId,
				'Row__c':row,
				'Column__c':column,
				'Value__c':value 	}))
		self.currentQMatrixCellId = response[23:41]
		return self.currentQMatrixCellId

	def insertQMatrix(self, qMatrix, rows, cols) :
		cellList = []
		for row in range(rows) :
			for col in range(cols) :
				newCell = QMatrixCell(str(self.currentIterationId),str(row),str(col),str(qMatrix[row][col]))
				cellList.append(newCell.toDictionary())
		self.salesforce.bulk.Q_Matrix_Cell__c.insert(cellList)

# main

rewards = [0,0,0,0]

rewards[CENTER] = REWARD
rewards[LEFT] = LOW_PENALTY
rewards[RIGHT] = LOW_PENALTY
rewards[LOST] = PENALTY

states = [[1,120],[121,160],[161,240],[0,0]]
actions = [[0,100],[100,100],[100,0]]

pid1Gains = [0.05,0.00005,0.5]
pid2Gains = [0.05,0.00005,0.5]

salesforce = SalesforceIntegration('matsfranco@personalforce.com','E2msf18@','00Df4000000kfbV','MlvcxdEfUUkfhyEWOmawJo7rw')
salesforce.startSession()

logger = DataLogger(salesforce)

mcu = MCU("MCU",'/dev/ttyUSB0',115200,pid1Gains,pid2Gains)
mcu.connect()

CAM = 1

agent = QLearning('Vision Q-learning',mcu,240,150,0.1,0.9,states,actions,rewards,CAM,1,logger)

lineDetector = AdaptativeLineDetector(240,60,60,240,60,agent)

lineDetector.learnHowToFollow(500)

#lineDetector.getLinePosition()

lineDetector.consolidateLearning()

x = raw_input()

lineDetector.runAsLearned()

mcu.stopMotors()


