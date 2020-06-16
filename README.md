# Adaptative Line Follower

The Adaptative Line Follower (ALF) robot was my graduation project in Computer Engineering at Sao Paulo Federal University.
This is the source code of a line follower robot implemented using Image Processing (OpenCV) to identify the line and an Reinforcement Learning (Q-learning) algorithm.


![ALF Robot](/images/alf-robot.jpg)

The Adaptative Linefollower Robot (ALF) platform. Built from zero using 3D printed parts.

## Implementation Highlights

### CVU_Q_Learning
This code is responsible for implementing image processing and Q-learning algorith using Python, OpenCV and a SQL Database to store learning process data


### MotorControlUnit
This is the source code of a Arduino-based module to control direct current electrical motors using PID controllers. 
It was implemented a Serial interface to implement the communication between CVLU and this unit.
CVLU send a stream of instructions to control robot movement.

### Webserver
This module implements a webserver using Pyhton and Flask to create a visual interface to follow up learning processes in real time and access previous learning processes for analysis using tables and chats in Javascript (Plot.ly).

### Diagrams

![Block Diagram](/images/block-diagram.png)

Block Diagram of the final system


![Class Diagram](/images/class-diagram.png)

Class Diagram with most important classes


![Database Diagram ](/images/database-er.png)

Database ER diagram.
