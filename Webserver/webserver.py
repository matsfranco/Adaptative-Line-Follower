from flask import Flask, Markup, render_template, request
import mysql.connector

rows = 0
columns = 0

qTableLog = []
qTableLog2 = []
LinePositionLog = []
iterationLog = []
currentStateLog = []
actionTakenLog = []
nextStateLog = []
rewardLog = []
positionErrorLog = []
iterationNumber = 0
heatMapTable = []
rewardRateLog = []
punishRateLog = []
averageReward = []
learningProcessDataArray = []

mydb = mysql.connector.connect(host='localhost',database='ALF',user='root',password='')
cursor = mydb.cursor()

solidColorByOrder = ['rgb(150, 200, 60,1)','rgb(24, 195, 240,1)','rgb(250, 215, 45,1)','rgb(255, 140, 40,1)','rgb(230, 65, 50,1)','rgb(255, 255, 255,1)']
transpColorByOrder = ['rgb(150, 200, 60,0.4)','rgb(24, 195, 240,0.4)','rgb(250, 215, 45,0.4)','rgb(255, 140, 40,0.4)','rgb(230, 65, 50,0.4)','rgb(255, 255, 255,0.4)']

def retrieveLearningProcessData(LearningProcessId) :
	global mydb
	global cursor
	global rows
	global columns
	lpData = ("SELECT NumberOfStates,NumberOfActions FROM LearningProcess WHERE Id =  %s" % LearningProcessId)
	cursor.execute(lpData)
	learningProcess = list(cursor.fetchall())
	rows = learningProcess[0][0]
	columns = learningProcess[0][1]
	print "row: "+str(rows)+" columns: "+str(columns)
	query = ("SELECT * FROM Iteration WHERE LearningProcess = %s" % LearningProcessId)
	cursor.execute(query)
	buildLogArrays(list(cursor.fetchall()))

def retrieveLastLearningProcess() :
	global mydb
	global cursor
	query = ("SELECT MAX(Id) FROM LearningProcess WHERE Status = 'DONE' ORDER BY Id DESC")
	cursor.execute(query)
	ids = int(filter(str.isdigit,str(cursor.fetchall())))
	return str(ids)

def retrieveCurrentLearningProcess() :
	global mydb
	global cursor
	global rows
	global columns
	learningProcess = []
	query = ("SELECT MAX(Id),NumberOfStates,NumberOfActions FROM LearningProcess WHERE Status = 'ON GOING' ")
	cursor.execute(query)
	learningProcess = list(cursor.fetchall())
	ids = learningProcess[0][0]
	rows = learningProcess[0][1]
	columns = learningProcess[0][2]
	
	print "row: "+str(rows)+" columns: "+str(columns)
	return str(ids)

def clearLogBuffers() :
	del qTableLog[:]
	del LinePositionLog[:]
	del iterationLog[:]
	del currentStateLog[:]
	del actionTakenLog[:]
	del nextStateLog[:]
	del rewardLog[:]
	del positionErrorLog[:]

def initialize2DArrayExtended() :
	global rows
	global columns
	matrix = [[0 for x in range(columns+2)] for y in range(rows)]
	return matrix

def initialize2DArray() :
	global rows
	global columns
	matrix = [[0 for x in range(columns)] for y in range(rows)]
	return matrix

def initialize3DArray() :
	global rows
	global columns
	global iterationNumber
	matrix = [[[0 for z in xrange(iterationNumber)] for x in range(columns)] for y in range(rows)]
	return matrix

def buildQTableLog2(cells,it) :
	global qTableLog2
	global rows
	global columns
	i = 0
	for r in range(rows) :
		for c in range(columns) :
			qTableLog2[r][c][it] = (float(cells[i]))
			i = i + 1

def buildQTable(cells) :
    global rows
    global columns
    matrix = initialize2DArrayExtended()
    i = 0
    for r in range(rows) :
        for c in range(1,columns+1) :
            matrix[r][c] = float(cells[i])
            i = i + 1
    return matrix	

def buildLogArrays(rawLogs) :
	global qTableLog
	global qTableLog2
	global LinePositionLog
	global iterationLog
	global currentStateLog
	global actionTakenLog
	global nextStateLog
	global rewardLog
	global positionErrorLog
	global iterationNumber
	global convergenceRateLog
	clearLogBuffers();
	it = 0
	iterationNumber = len(rawLogs)
	qTableLog2 = initialize3DArray()
	for rawLog in rawLogs :
 		splitedValues = rawLog[3].split(',')
		buildQTableLog2(splitedValues,it)
		qTableLog.append(buildQTable(splitedValues))
		LinePositionLog.append(int(rawLog[4]))
		currentStateLog.append(int(rawLog[5]))
		actionTakenLog.append(int(rawLog[6]))
		nextStateLog.append(int(rawLog[7]))
		rewardLog.append(float(rawLog[8]))
		positionErrorLog.append(abs(int(rawLog[4])-120))
		iterationLog.append(str(it))
		it = it + 1

def buildHeatmapTable() : 
	global currentStateLog
	global actionTakenLog
	global heatMapTable
	global rows
	global columns
	maxSize = len(currentStateLog)
	print 'rows '+str(rows)+'columns '+str(columns)
	heatMapTable = initialize2DArray()
	for it in range(maxSize) :
		row = currentStateLog[it]
		column = actionTakenLog[it]
		heatMapTable[row][column] = heatMapTable[row][column] + 1 

def buildConvergenceRateLog() :
	rewardCount = 0.0
	rewardPercent = 0.0
	punishCount = 0.0
	punishPercent = 0.0
	global rewardLog

	global rewardRateLog
	global punishRateLog
	rewardRateLog.append(0.0)
	punishRateLog.append(0.0)

	maxSize = len(rewardLog)
	for it in range(1,maxSize) :
		if rewardLog[it] > 0.0 : 
			rewardCount = rewardCount + 1.0
			rewardRateLog.append(rewardCount/float(it))
		else : 
			punishCount = punishCount + 1.0
			punishRateLog.append(punishCount/float(it))

def buildAverageReward() :

	global rewardLog	
	global averageReward
	maxSize = len(rewardLog)
	cumulativeSum = 0.0
	for it in range(maxSize) :
		cumulativeSum = cumulativeSum + rewardLog[it]
		mean = round(cumulativeSum/float(it+1),4)
		averageReward.append(mean)

def connectToDatabase() :
	global mydb
	global cursor
	mydb = mysql.connector.connect(host='localhost',database='ALF',user='root',password='')
	cursor = mydb.cursor()

def disconnectFromDatabase() :
	global mydb
	global cursor
	cursor.close()
	mydb.close()

def retrieveLearningProcesses() :
	global mydb
	global cursor
	global learningProcessDataArray
	query = ("SELECT Id,Status,CreatedDate,NumberOfStates,NumberOfActions,Rewards,MaxSpeed,Alpha,Lambda,ImageWidth,ImageHeight,ImageFps,NumberOfIterations FROM LearningProcess")
	cursor.execute(query)
	buildLearningProcessesDataArray(list(cursor.fetchall()))

def buildLearningProcessesDataArray(learningProcessesData) :

	del learningProcessDataArray[:]
	for i in range(len(learningProcessesData)) :
		learningProcessLine = []
		for j in range(13) :
			print "i,j = "+str(i) +","+ str(j)
			learningProcessLine.append(learningProcessesData[i][j])
		learningProcessDataArray.append(learningProcessLine)


log = []

app = Flask(__name__)

@app.route('/processList')
def getLearningProcessList():
	global learningProcessDataArray
	connectToDatabase()
	retrieveLearningProcesses()
	disconnectFromDatabase()
	return render_template('processList.html',
		learningProcessesData = learningProcessDataArray,
		numberOfRows = len(learningProcessDataArray),
		numberOfColumns = 13)

@app.route('/processhistory')
def processHistory():
	return render_template('searchPage.html')

@app.route('/processhistory',methods=['POST'])
def processHistory_POST():
	connectToDatabase()
	LearningProcess = request.form['lpId']
	retrieveLearningProcessData(LearningProcess)
	disconnectFromDatabase()
	buildHeatmapTable()
	buildAverageReward()
	print qTableLog[len(qTableLog)-1]
	return render_template('processHistoryCharts.html',
		learningProcessId = LearningProcess, 
		dataColor=solidColorByOrder,
		underDataColor=transpColorByOrder,
		chart1_title='Action choice probability per State', 
    	stateActionValues = qTableLog[len(qTableLog)-1],
    	generalMeasures_title ='Q-learning general data per iteration',
    	chart2_linePosition = LinePositionLog,
    	chart2_currentState = currentStateLog,
    	chart2_actionTaken = actionTakenLog,
    	chart2_nextState = nextStateLog,
    	chart2_reward = rewardLog,
    	chart2_positionError = positionErrorLog,
    	chart2_labels = iterationLog,
		states = rows,
		actions = columns,
		stateActionProbLog = qTableLog2,
		heatmap_data = str(heatMapTable),
		averageReward_data = str(averageReward)	)


@app.route('/currentprocess')
def currentProcess():
	connectToDatabase()
	LearningProcess = retrieveCurrentLearningProcess()
	retrieveLearningProcessData(LearningProcess)
	state = 0
	disconnectFromDatabase()
	buildHeatmapTable()
	buildAverageReward()
	return render_template('currentProcessCharts.html',
		title = 'Current Learning Process: ',
		learningProcessId = LearningProcess, 
		dataColor=solidColorByOrder,
		underDataColor=transpColorByOrder,
		chart1_title='Action choice probability per State', 
    	stateActionValues = qTableLog[len(qTableLog)-1],
    	generalMeasures_title ='Q-learning general data per iteration',
    	chart2_linePosition = LinePositionLog,
    	chart2_currentState = currentStateLog,
    	chart2_actionTaken = actionTakenLog,
    	chart2_nextState = nextStateLog,
    	chart2_reward = rewardLog,
    	chart2_positionError = positionErrorLog,
    	chart2_labels = iterationLog,
    	chart3_state = 1,
		states = rows,
		actions = columns,
		stateActionProbLog = qTableLog2,
		heatmap_data = str(heatMapTable),
		averageReward_data = str(averageReward)	)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050)