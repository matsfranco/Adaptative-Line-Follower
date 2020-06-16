#include "Encoder.h"
#include "Motor.h"
#include "SpeedControl.h"
#include <math.h>
#include <TimerOne.h>
#define ENCODER_PPR 341
#define dT  20000

#define BATT_CELL_0 A0
#define BATT_CELL_1 A1
#define BATT_CELL_2 A2

int command = 0;
bool printMeasures = false;
bool connectedToController = false;

#define M1_D    13
#define M1_R    4
#define M1_PWM  5
#define M1_A    3
#define M1_B    7
#define M1_THRUST_LEVER A1


Motor   M1(M1_D,M1_R,M1_PWM);
Encoder E1(M1_A,M1_B,dT,ENCODER_PPR);
SpeedControl SpeedController1(&M1,&E1);
int M1_SPD = 0;
int M1_POT = 0;
double kP_1 = 0.05, kI_1 = 0.00005, kD_1 = 0.5;
//double kP_1 = 0.1, kI_1 = 0.0, kD_1 = 0.0;

#define M2_D    12
#define M2_R    11
#define M2_PWM  6
#define M2_A    2
#define M2_B    8
#define M2_THRUST_LEVER A1

Motor M2(M2_D,M2_R,M2_PWM);
Encoder E2(M2_A,M2_B,dT,ENCODER_PPR);
SpeedControl SpeedController2(&M2,&E2);
int M2_SPD = 0;
int M2_POT = 0;
double kP_2 = 0.05, kI_2 = 0.00005, kD_2 = 0.5;
//double kP_2 = 0.1, kI_2 = 0.0, kD_2 = 0.0;


/* Command List */
#define SET_SPEEDS              1000
#define SET_SPDC1_COEF          3001
#define SET_SPDC2_COEF          3002
#define STOP                    3003
#define ENABLE_SPEED_MEASURE    4001
#define DISABLE_SPEED_MEASURE   4002
#define HANDSHAKE               3000
#define SUCCESS                 5000
#define FAIL                    5001


void executeCommand(int comm) {
      
    switch(comm) {
      case SET_SPEEDS:
        M1_POT = Serial.parseInt();
        M2_POT = Serial.parseInt();
        SpeedController1.setSpeed(M1_POT*6);
        SpeedController2.setSpeed(M2_POT*6);
        Serial.print(M1_POT);
        Serial.print(", ");
        Serial.println(M2_POT);
        break;
    
      case STOP:
        SpeedController1.setSpeed(0);
        SpeedController2.setSpeed(0);
        SpeedController1.resetController();
        SpeedController2.resetController();
        //Serial.println(5000);
        break;

      case SET_SPDC1_COEF:
        kP_1 = Serial.parseFloat()/1000;
        kI_1 = Serial.parseFloat()/1000;
        kD_1 = Serial.parseFloat()/1000;
        //Serial.print("New SPDC1 coeficients: "); Serial.print(kP_1); Serial.print(","); Serial.print(kI_1); Serial.print(","); Serial.println(kD_1);
        //Serial.println(5000);
        break;
    
      case SET_SPDC2_COEF:
        kP_2 = Serial.parseFloat()/1000;
        kI_2 = Serial.parseFloat()/1000;
        kD_2 = Serial.parseFloat()/1000;
        //Serial.print("New SPDC2 coeficients: "); Serial.print(kP_2); Serial.print(","); Serial.print(kI_2); Serial.print(","); Serial.println(kD_2);
        //Serial.println(5000);
        break;
        
      case ENABLE_SPEED_MEASURE:
        printMeasures = true;
        //Serial.println(5000);
        break;
      case DISABLE_SPEED_MEASURE:
        printMeasures = false;
        //Serial.println(5000);
        break;
      case HANDSHAKE:
        connectedToController = true;
        Serial.println(comm);
        break;
      default:
        if(comm != 0) Serial.println(FAIL);
        break;
    }
  }


void setup() {
  Serial.begin(115200);
  pinMode(BATT_CELL_0,INPUT);
  pinMode(BATT_CELL_1,INPUT);
  pinMode(BATT_CELL_2,INPUT);
  Timer1.initialize(dT);
  Timer1.attachInterrupt(getMotorSpeeds);
  attachInterrupt(1, Encoder_1_Event, CHANGE);
  attachInterrupt(0, Encoder_2_Event, CHANGE);
  SpeedController1.setGains(kP_1,kI_1,kD_1);
  SpeedController2.setGains(kP_2,kI_2,kD_2);
  SpeedController1.setSpeed(0);
  SpeedController2.setSpeed(0);
}

void loop() {
  while(Serial.available() > 0) {
    executeCommand(Serial.parseInt());
  }
}

void getMotorSpeeds() {
  noInterrupts();
  SpeedController1.adjustPWM();
  SpeedController2.adjustPWM();
  float BATT =  analogRead(BATT_CELL_0)*(5.0 / 1023.0) + 
                analogRead(BATT_CELL_1)*(5.0 / 1023.0) + 
                analogRead(BATT_CELL_2)*(5.0 / 1023.0);  

  if(printMeasures) {
    Serial.print(SpeedController1.getSpeed()/6); Serial.print(","); 
    Serial.print(SpeedController2.getSpeed()/6); Serial.print(","); 
    Serial.print(SpeedController1.getITerm()); Serial.print(","); 
    Serial.print(SpeedController2.getITerm()); Serial.print(",");
    Serial.print(SpeedController1.getPWM()); Serial.print(","); 
    Serial.print(SpeedController2.getPWM()); Serial.print(","); 
    Serial.println(BATT); 
    
  }
  interrupts();
}

void Encoder_1_Event() {
  E1.updateCount();
}
void Encoder_2_Event() {
  E2.updateCount();
}
