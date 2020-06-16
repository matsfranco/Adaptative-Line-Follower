import math
import random
import sys
import cv2
import numpy as np
import time
import serial
import mysql.connector
from picamera import PiCamera
from picamera.array import PiRGBArray
from datetime import date,datetime,timedelta

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
	#	2. Mateus Franco @ 29/10/2018
	#		dataLogger removed
	def __init__(self,name,MCU,imageWidth,maxSpeed,ALPHA,LAMBDA,states,actions,rewards,camer,actionDelay,database) :
		self.learningPhase = True
		self.maxSpeed = maxSpeed
		self.numberOfActions = len(actions)
		self.numberOfStates = len(states)
		self.MCU = MCU
		self.P = []	
		self.Q = []
		self.currentState = 0
		self.nextState = 0
		self.currentAction = 0
		self.action = 0
		self.ALPHA = ALPHA
		self.LAMBDA = LAMBDA
		self.states = states
		self.actions = actions
		self.rewards = rewards
		self.name = name
		self.message = ''
		self.actionDelay = actionDelay
		print str(len(actions))
		print str(len(states))
		self.initQAndP()
		self.database = database
		print '>> QLearning - New QLearning defined: '+self.name+'\n'


	# Method Name: initQAndP
	# Description: Iniialize Q table with zeros
	# Created by Mateus Franco @ 02/10/2018
	# Change History:
	def initQAndP(self) :
		self.P = [0 for x in range(self.numberOfActions)] 
		self.Q = [[0 for x in range(self.numberOfActions)] for y in range(self.numberOfStates)]
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
		print ">> QLearning - Actions["+str(action)+"] selected - "+str(self.actions[action])
		self.MCU.setSpeeds(self.actions[action][M1],self.actions[action][M2])
		print ">> QLearning - Action taken = "+str(action)
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
		for s in range (len(self.states) - 1) :
			if centerPosition >= self.states[s][MIN] and centerPosition <= self.states[s][MAX] :
				return s
		return len(self.states) - 1

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
			if(lineCenter != None) : self.currentLinePosition = lineCenter 
			else : self.currentLinePosition = 0
			self.currentState = self.getState(self.currentLinePosition)
			self.currentAction = self.selectAction(self.currentState)
			print ">> QLearning - Line Center = "+str(lineCenter)+"\tCurrent State = "+str(self.currentState)
			self.takeAction(self.currentAction)
			time.sleep(0.3)
		else :
			if(lineCenter != None) : self.nextLinePosition = lineCenter
			else : self.nextLinePosition = 0
			self.nextState = self.getState(self.nextLinePosition)
			self.reward = self.rewards[self.nextState]
			self.Q[self.currentState][self.currentAction] = ((1.0 - self.ALPHA) * self.Q[self.currentState][self.currentAction] +self.ALPHA * (self.reward + self.LAMBDA * self.max_a_Q(self.nextState)))
			self.MCU.stopMotors()
			database.insertNewIteration(iteration,self.Q,self.currentLinePosition,self.currentState,self.currentAction,self.nextState,self.reward)# Send to database
			self.currentState = self.nextState
			print ">> QLearning - Line Center = "+str(lineCenter)+"\tNext State = "+str(self.currentState)

# MCU Class
# Description: 	Implements all structure necessary connect, send and receive informations from the 
#				Motor Control Unit (MCU) attached to a specific serial connection
# Created by Mateus Franco @ 06/10/2018+ 
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
		print ">> MCU - New Serial Device: "+name+ " created as MCU @ "+str(port)+ " / "+str(baudRate)+ "bps\n"
	
	# Method Name: recvMessage 
	# Description: Receive a string from MCU and return this message as a string without \r\n in the end
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def recvMessage(self) :
		message = self.port.readline()
		message = message.replace('\r\n','')
		print ">> MCU - Received from "+self.connectionName+": "+str(message)+"\n"
		return message
	
	# Method Name: connect
	# Description: 	Create a serial port and estabilshes the serial connection between CVU and MCU apply a
	#				handshade kind protocol. After send the connection message waits an echo as response.
	#				If resonse don't come, another attemp is made periodically
	# Created by Mateus Franco @ 06/10/2018
	# Change History:
	def connect(self) :
		print ">> MCU - Connecting to MCU...\n"
		self.port = serial.Serial(self.port,self.baudRate)
		time.sleep(2)
		self.sendMessage("3000\n")
		time.sleep(0.1)
		while (int(self.recvMessage()) != 3000) : 
			print ">> MCU - Failed to connect to MCU. Trying again!\n"
		print ">> MCU - MCU connected!\n"
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
			print ">> MCU - sent to "+self.connectionName+" - "+message
		else :
			print ">> MCU - Command already sent\n"
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
		time.sleep(0.01)
		self.sendMessage("3003\n");
		time.sleep(0.01)
		self.sendMessage("3003\n");
		time.sleep(0.01)
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
	def __init__(self,width,height,frameRate,windowWidth,windowHeight,agent,iterationNumber) :
		self.height = height
		self.width = width
		self.iterationNumber = iterationNumber
		self.windowHeight = windowHeight
		self.windowWidth = windowWidth
		self.camera = PiCamera()
		self.camera.resolution = (self.width,self.height)
		self.camera.framerate = frameRate
		self.rawCapture = PiRGBArray(self.camera,size=(self.width,self.height))
		time.sleep(0.1)
		self.agent = agent
		self.database = agent.database
		self.stateActionMap = []
		print '>> AdaptativeLineDetector - New AdaptativeLineDetector defined: '+str(self.camera)+' with agent '+self.agent.name+'\n'
		self.agent.database.createNewLearningProcess(
			self.agent.numberOfStates,str(self.agent.states),self.agent.numberOfActions,
			str(self.agent.actions),str(self.agent.rewards),self.agent.ALPHA,
			self.agent.LAMBDA,self.iterationNumber,width,height,frameRate,self.agent.maxSpeed)

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
	# Description:	Obtain a new frame in each iteractionCal, define QLearning step and calls QLearning methods
	#				to update states, actions and Q table
	# Created by Mateus Franco @ 03/10/2018
	# Change History:
	#	08/10/2018 - Mateus Franco
	#		raw_input call to stop running to easy practical test routine
	def learnHowToFollow(self) :
		step = 0
		iteration = 1
		self.agent.database.startLearningProcess()
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
				#x = raw_input()

			self.rawCapture.truncate(0)
			iteration = iteration + 0.5
			if(iteration > self.iterationNumber) : break
		self.agent.learningPhase = False
		self.agent.printQTable(iteration)

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
		print ">> AdaptativeLineDetector - Final State - Action Map"
		for state in range(len(self.agent.states)) :
			print "["+str(state)+"] - "+str(self.stateActionMap[state])
		self.database.closeCurrentLearningProcess()
		
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

# DatabaseIntegrator Class
# Description: 	
# Created by Mateus Franco @ 06/11/2018
# Change History:
class DatabaseIntegrator :

	def __init__(self,host,database,user,password) :
		self.host = host
		self.database = database
		self.user = user 
		self.password = password
		self.currentLearningProcessId = 0
		print '>> DatabaseIntegrator.init() - New DatabaseIntegrator defined: '+self.host+' on database '+self.database+' as '+self.user

	def connectToDatabase(self) :
		self.mydb = mysql.connector.connect(host=self.host,database=self.database,user=self.user,password=self.password)
		self.cursor = self.mydb.cursor()
		print '>> DatabaseIntegrator - Connection Opened - '+str(self.mydb)

	def disconnectFromDatabase(self) :
		self.mydb.commit()
		self.cursor.close()
		self.mydb.close()
		print '>> DatabaseIntegrator - Commited. Connection Closed'

	def closePreviousLearningProcess(self) :
		print 'DatabaseIntegrator: Closing previous opened Learning Processes...'
		now = (datetime.now() + timedelta(hours=-5))
		nowFormated = now.strftime('%Y-%m-%d %H:%M:%S')
		self.connectToDatabase();
		query = ("UPDATE LearningProcess SET Status = 'DROPED', ClosingDate = '%s' WHERE Status = 'ON GOING' OR Status = 'OPEN' " % nowFormated)
		self.cursor.execute(query)
		print '>> DatabaseIntegrator - Closing previous opened Learning Processes - FINISHED'
		self.disconnectFromDatabase();

	def retrieveNewLearningProcessId(self) :
		query = ("SELECT MAX(Id) FROM LearningProcess WHERE Status = 'OPEN' ")
		self.cursor.execute(query)
		ids = int(filter(str.isdigit,str(self.cursor.fetchall())))
		self.disconnectFromDatabase();
		return ids

	def createNewLearningProcess(self,NumberOfStates,States,NumberOfActions,Actions,Rewards,Alpha,Lambda,NumberOfIterations,ImageWidth,ImageHeight,ImageFps,MaxSpeed) :
		self.connectToDatabase();
		self.NumberOfActions = NumberOfActions
		self.NumberOfStates = NumberOfStates
		query = "INSERT INTO LearningProcess(NumberOfStates,States,NumberOfActions,Actions,Rewards,Alpha,Lambda,NumberOfIterations,ImageWidth,ImageHeight,ImageFps,MaxSpeed,Status,CreatedDate) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)"
		args = (NumberOfStates,States,NumberOfActions,Actions,Rewards,Alpha,Lambda,NumberOfIterations,ImageWidth,ImageHeight,ImageFps,MaxSpeed,'OPEN',(datetime.now() + timedelta(hours=-5)))
		self.cursor.execute(query,args)
		self.disconnectFromDatabase();
		self.connectToDatabase();
		self.currentLearningProcessId = self.retrieveNewLearningProcessId();
		print '>> DatabaseIntegrator - new LearningProcess created Id = '+str(self.currentLearningProcessId)

	def startLearningProcess(self) :
		self.connectToDatabase();
		query = ("UPDATE LearningProcess SET Status = 'ON GOING' WHERE Status = 'OPEN' AND Id = %s" % self.currentLearningProcessId)
		self.cursor.execute(query)
		self.disconnectFromDatabase();
		print ">> DatabaseIntegrator - LearningProcess "+str(self.currentLearningProcessId)+" Status changed FROM 'OPEN' TO 'ON GOING' "

	def convertMatrixToString(self,qTable) :
		cellList = []
		for row in range(self.NumberOfStates) :
			for col in range(self.NumberOfActions) :
				cellList.append(round(qTable[row][col],2))		
		cellListS = str(cellList)
		return cellListS[1:len(cellListS) -1]

	def insertNewIteration(self,IterationNumber,qTable,LinePosition,CurrentState,ActionTaken,NextState,Reward) :
		qTableList = self.convertMatrixToString(qTable)
		self.connectToDatabase()
		query = "INSERT INTO Iteration(LearningProcess,IterationNumber,qTable,LinePosition,CurrentState,ActionTaken,NextState,Reward) VALUES (%s,%s,%s,%s,%s,%s,%s,%s)"
		args = (self.currentLearningProcessId,IterationNumber,qTableList,LinePosition,CurrentState,ActionTaken,NextState,Reward)
		self.cursor.execute(query,args)	
		self.disconnectFromDatabase()

	def closeCurrentLearningProcess(self) :
		print '>> DatabaseIntegrator- Closing current Learning Processes...'
		now = (datetime.now() + timedelta(hours=-5))
		nowFormated = now.strftime('%Y-%m-%d %H:%M:%S')
		self.connectToDatabase();
		query = ("UPDATE LearningProcess SET Status = 'CLOSED', ClosingDate = '%s' WHERE Status = 'ON GOING' " % nowFormated)
		self.cursor.execute(query)
		print '>> DatabaseIntegrator - Current Learning Process Closed.'
		self.disconnectFromDatabase();

# LearningConfiguration Class
# Description: 	
# Created by Mateus Franco @ 02/12/2018
# Change History:
class LearningConfiguration :

	def __init__(self, desc, states, actions, rewards,a,l) :
		self.description = desc
		self.states = states
		self.actions = actions
		self.rewards = rewards
		self.Alpha = a
		self.Lambda = l

# main

print "-------------------------------------------------------"
print "-------------------------------------------------------"
print "-----------------      ALF Robot      -----------------"
print "-------------------------------------------------------"
print "-------------------------------------------------- v1.6\n"

print ">> main() - Creating standard Learning Configurations..."

learningConfigurations = []

learningConfigurations.append(LearningConfiguration(	"BASIC A: 4 states, 3 actions",
									[[1,101],[102,139],[140,240],[0,0]],
									[[0,100],[100,100],[100,0]],
									[-0.07,1,-0.07,-1],
									0.1,0.9))

learningConfigurations.append(LearningConfiguration(	"BASIC B: 4 states, 3 actions, less wide center state",
									[[1,111],[112,129],[130,240],[0,0]],
									[[0,100],[100,100],[100,0]],
									[-0.07,1,-0.07,-1],
									0.1,0.9))


learningConfigurations.append(LearningConfiguration(	"INTERMEDIATE A: 6 states, 5 actions (with reverse)",
											[[1,61],[62,100],[101,140],[141,179],[180,240],[0,0]],
											[[-100,100],[0,100],[100,100],[100,0],[100,-100]],
											[-0.05,-0.02,1.0,-0.02,-0.05,-1],
											0.1,0.9))

learningConfigurations.append(LearningConfiguration(	"INTERMEDIATE B: 6 states, 5 actions, smooth turns",
											[[1,101],[102,116],[117,124],[125,139],[140,240],[0,0]],
											[[0,100],[50,100],[100,100],[100,50],[100,0]],
											[-0.05,-0.02,1.0,-0.02,-0.05,-1],
											0.1,0.9))

learningConfigurations.append(LearningConfiguration("ENHANCED A: 8 states, 7 actions (with reverse)",
										[[1,31],[32,62],[63,99],[100,141],[142,178],[179,209],[210,240],[0,0]],
										[[-120,120],[0,120],[80,120],[120,120],[120,80],[120,0],[120,-120]],
										[-0.07,-0.05,-0.02,1.0,-0.02,-0.05,-0.07,-1],0.1,0.9))

learningConfigurations.append(LearningConfiguration("ENHANCED A: 8 states, 7 actions (with reverse)",
										[[1,17],[18,34],[35,51],[52,68],[69,85],[86,102],[103,119],[120,136],[137,153],[154,170],[171,187],[188,204],[205,221],[222,240],[0,0]],
										[[0,120],[50,120],[60,120],[70,120],[80,120],[90,120],[100,120],[110,120],[120,120],[120,110],[120,100],[120,90],[120,80],[120,70],[120,60],[120,50],[120,0]],
										[-0.3,-0.25,-0.2,-0.15,-0.1,-0.05,1.0,1.0,-0.05,-0.1,-0.15,-0.2,-0.25,-0.3,-1],0.1,0.9))


print ">> main() - "+str(len(learningConfigurations))+" Learning Configurations were created.\n"

pid1Gains = [0.05,0.00005,0.5]
pid2Gains = [0.05,0.00005,0.5]

print ">> main() - Calling Database Integration connection routine...\n"

database = DatabaseIntegrator('localhost','ALF','root','')

print ">> main() - Calling MCU Integration connection routine...\n"

mcu = MCU("MCU",'/dev/ttyUSB0',115200,pid1Gains,pid2Gains)
mcu.connect()

CAM = 1

print ">> main() - Please, choose a stardard Learning Process configuration:\n"
for i in range(len(learningConfigurations)) :
	print str(i)+". "+learningConfigurations[i].description
op = int(raw_input('\nType the corresponding number: '))

print ">> main() - Learning Configuration choosed: "+str(op)+", "+learningConfigurations[op].description+"\n"

print ">> main() - Creating Learning Agent based on configuration "+str(op)+"\n"
agent = QLearning('Vision Q-learning',mcu,240,150,
								learningConfigurations[op].Alpha,
								learningConfigurations[op].Lambda,
								learningConfigurations[op].states,
								learningConfigurations[op].actions,
								learningConfigurations[op].rewards,CAM,1,database)

print ">> main() - Closing previous unconcluded Learning Processes"
database.closePreviousLearningProcess()

it = input('>> main() - Please, type how many iteractions this Learning Process will iterate: ')

print ">> main() - Creating the Adaptativa Line Detector object..."
lineDetector = AdaptativeLineDetector(240,64,60,240,64,agent,it)


print ">> main() - Starting Learning Process"

lineDetector.learnHowToFollow()

#lineDetector.getLinePosition()

print ">> main() - Learning Process Finished. Stopping motors..."
mcu.stopMotors()
print ">> main() - Ready to consolidate the learning process. Type ENTER to start."
x = raw_input()
lineDetector.consolidateLearning()
print ">> main() - Learning Process consolidated. Type ENTER to start running as learned."
x = raw_input()
lineDetector.runAsLearned()

mcu.stopMotors()


