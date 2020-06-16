from flask import Flask, Markup, render_template, request
import mysql.connector

rows = 4
columns = 3

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

mydb = mysql.connector.connect(host='localhost',database='ALF',user='root',password='')
cursor = mydb.cursor()

solidColorByOrder = ['rgb(150, 200, 60,1)','rgb(24, 195, 240,1)','rgb(250, 215, 45,1)','rgb(255, 140, 40,1)','rgb(230, 65, 50,1)','rgb(255, 255, 255,1)']
transpColorByOrder = ['rgb(150, 200, 60,0.4)','rgb(24, 195, 240,0.4)','rgb(250, 215, 45,0.4)','rgb(255, 140, 40,0.4)','rgb(230, 65, 50,0.4)','rgb(255, 255, 255,0.4)']

def retrieveLearningProcessData(LearningProcessId) :
    global mydb
    global cursor
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
	query = ("SELECT MAX(Id) FROM LearningProcess WHERE Status = 'ON GOING' ")
	cursor.execute(query)
	ids = int(filter(str.isdigit,str(cursor.fetchall())))
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
	maxSize = len(currentStateLog)
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

actionNames = ['','Turn Left', 'Go Ahead','Turn Right','']

log = []

app = Flask(__name__)

@app.route('/probhistory')
def stateProbabilitesAndGeneralMeasures():
	connectToDatabase()
	LearningProcess = retrieveLastLearningProcess()
	retrieveLearningProcessData(LearningProcess)
	maxSize = len(iterationLog)
	disconnectFromDatabase()
	if (maxSize > 50) : firstIt = maxSize - 50
	else : firstIt = 0
	global log
	return render_template('bar_chart.html', 
    	chart1_title='Action choice probability per State', 
    	chart1_max=100,    	
    	chart1_labels = actionNames, 
    	chart1_dataset = qTableLog[len(qTableLog)-1],
    	chart2_title ='Q-learning measures per Iteration',
    	chart2_linePosition = LinePositionLog[firstIt:maxSize],
    	chart2_currentState = currentStateLog[firstIt:maxSize],
    	chart2_actionTaken = actionTakenLog[firstIt:maxSize],
    	chart2_nextState = nextStateLog[firstIt:maxSize],
    	chart2_reward = rewardLog[firstIt:maxSize],
    	chart2_positionError = positionErrorLog[firstIt:maxSize],
    	chart2_labels = iterationLog[firstIt:maxSize])

@app.route('/probhistory', methods=['POST'])
def stateProbabilitesAndGeneralMeasures_POST():
	connectToDatabase()
	firstIteration = request.form['firstIt']
	lastIteration = request.form['lastIt']
	learningProcessId = request.form['lpId']
	firstIt = int(firstIteration)
	lastIt = int(lastIteration)
	lpId = str(learningProcessId)
	retrieveLearningProcessData(lpId)
	disconnectFromDatabase()
	return render_template('bar_chart.html', 
		chart1_title='Action choice probability per State', 
		chart1_max=100,    	
		chart1_labels = actionNames, 
		chart1_dataset = qTableLog[lastIt],
		chart2_title ='Q-learning measures per Iteration',
		chart2_linePosition = LinePositionLog[firstIt:lastIt],
		chart2_currentState = currentStateLog[firstIt:lastIt],
		chart2_actionTaken = actionTakenLog[firstIt:lastIt],
		chart2_nextState = nextStateLog[firstIt:lastIt],
		chart2_reward = rewardLog[firstIt:lastIt],
		chart2_positionError = positionErrorLog[firstIt:lastIt],
		chart2_labels = iterationLog[firstIt:lastIt])

@app.route('/probevolution')
def actionProbabilitiesEvolution():
	connectToDatabase()
	LearningProcess = retrieveLastLearningProcess()
	retrieveLearningProcessData(LearningProcess)
	disconnectFromDatabase()
	return render_template('probabilityEvolutionChart.html',
		solidDataColor=solidColorByOrder, 
    	chart2_title ='Action Choice Probability per State',
    	chart2_labels = iterationLog,
    	state = 0,
    	actions = columns,
    	log = qTableLog2)

@app.route('/probevolution', methods=['POST'])
def actionProbabilitiesEvolution_POST():
	connectToDatabase()
	learningProcessId = request.form['lpId']
	stateId = int(request.form['state'])
	lpId = str(learningProcessId)
	retrieveLearningProcessData(lpId)
	disconnectFromDatabase()
	return render_template('probabilityEvolutionChart.html',
		solidDataColor=solidColorByOrder,
		chart2_title ='Action Choice Probability per State',
		chart2_labels = iterationLog,
		state = stateId,
		actions = columns,
		log = qTableLog2)

@app.route('/currentprocess')
def currentProcess():
	connectToDatabase()
	LearningProcess = retrieveCurrentLearningProcess()
	retrieveLearningProcessData(LearningProcess)
	state = 0
	disconnectFromDatabase()
	buildHeatmapTable()
	#buildConvergenceRateLog()
	buildAverageReward()
	return render_template('currentProcessCharts.html',
		title = 'Current Learning Process: ',
		learningProcessId = LearningProcess, 
		dataColor=solidColorByOrder,
		underDataColor=transpColorByOrder,
		chart1_title='Action choice probability per State',     	
    	chart1_labels = actionNames, 
    	chart1_dataset = qTableLog[len(qTableLog)-1],
    	chart2_title ='Q-learning general data per iteration',
    	chart2_linePosition = LinePositionLog,
    	chart2_currentState = currentStateLog,
    	chart2_actionTaken = actionTakenLog,
    	chart2_nextState = nextStateLog,
    	chart2_reward = rewardLog,
    	chart2_positionError = positionErrorLog,
    	chart2_labels = iterationLog,
    	chart3_state = 1,
    	chart3_title ='Probability of choosing an action for State ',
		chart3_labels = iterationLog,
		chart3_states = rows,
		chart3_actions = columns,
		chart3_log = qTableLog2,
		heatmap_data = str(heatMapTable),
		averageReward_data = str(averageReward)	)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)