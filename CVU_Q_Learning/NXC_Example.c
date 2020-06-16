// Profesor: Francisco Zamora-Martinez
// Alumno:   Bernardo Gomez Vicent
// Fecha:    (Mayo 2012)
//
// Grado en Ingenería de Sistemas de Información
// Escuela de Enseñanzas Técnicas
// Universidad CEU-Cardenal Herrera

// ESTADOS
#define NUM_STATES 4
#define WW 0
#define WB 1
#define BW 2
#define BB 3
// ACCIONES
#define NUM_ACTIONS 3
#define ACTION_A 0 // ONE SIDE
#define ACTION_B 1 // OTHER SIDE
#define ACTION_C 2 // STRAIGHT
// PARAMETERS OF Q-LEARNING
#define ALPHA   0.5
#define LAMBDA 1.0
// AUXILIAR MACROS
#define max(a,b) ((a)<(b))?(b):(a)
#define min(a,b) ((a)<(b))?(a):(b)
#define COL1 0
#define COL2 30
#define COL3 60
#define RANDOM_PREC 10000
#define MOVE_WAIT_TIME 100
#define POWER 30
#define REWARD  +1.0
#define PENALTY -1.0
#define VERY_LITTLE_PENALTY -0.05

long WHITE;
long BLACK;

int Estado;
float Qmatrix[NUM_STATES][NUM_ACTIONS];
float rewards[NUM_STATES];

void MyNumOut(int col, int line, float value) {
  int int_value = value*100;
  NumOut(col, line, int_value);
}

// Busca una accion lanzando un dado con tantas caras como
// acciones. Se usa el metodo softmax para calcular la probabilidad de
// cada cara.
int selectAction(int s) {
  float sum = 0;
  float P[NUM_ACTIONS];
  float max_=Qmatrix[s][0], min_=Qmatrix[s][0];
  for (int i=0; i<NUM_ACTIONS; ++i) {
    if (Qmatrix[s][i] > max_) max_ = Qmatrix[s][i];
    if (Qmatrix[s][i] < min_) min_ = Qmatrix[s][i];
  }
  if (max_ - min_ > 30) min_ = max_ - 30;
  for (int i=0; i<NUM_ACTIONS; ++i) {
    P[i]  = exp(Qmatrix[s][i] - min_);
    sum  += P[i];
  }
  float inv_sum = 1.0/sum;
  float acum = 0;
  int r = Random(RANDOM_PREC);
  int action = NUM_ACTIONS-1;
  for (int i=0; i<NUM_ACTIONS; ++i) {
    P[i]  = P[i] * inv_sum;
    acum += P[i];
    if (r <= acum*RANDOM_PREC) {
      action = i;
      break;
    }
  }
  MyNumOut(0, LCD_LINE7,  P[0]);
  MyNumOut(25, LCD_LINE7, P[1]);
  MyNumOut(50, LCD_LINE7, P[2]);
  ClearLine(LCD_LINE5);
  NumOut(0, LCD_LINE5, r);
  return action;
}

//DETECTAR ESTADO SIGUIENTE
int leer() {
  long luz1 = 0;
  long luz2 = 0;
  luz1 = SensorRaw(IN_1);
  luz2 = SensorRaw(IN_2);
  NumOut(0,  LCD_LINE8, luz1);
  NumOut(60, LCD_LINE8, luz2);
  float luz1_white_dist = abs(WHITE - luz1);
  float luz1_black_dist = abs(BLACK - luz1);
  float luz2_white_dist = abs(WHITE - luz2);
  float luz2_black_dist = abs(BLACK - luz2);
  if (luz1_white_dist < luz1_black_dist &&
      luz2_white_dist < luz2_black_dist)
    return WW;
  else if (luz1_white_dist <  luz1_black_dist &&
	   luz2_black_dist <= luz2_white_dist)
    return WB;
  else if (luz1_black_dist <= luz1_white_dist &&
	   luz2_white_dist <  luz2_black_dist)
    return BW;
  else
    return BB;
}

//SUBRUTINA DE MOVIMIENTO EN BASE A LA ACCIÓN
void move(int action) {
  if (action==ACTION_A) {
    OnFwdReg(OUT_A,  POWER/2, OUT_REGMODE_SPEED);
    OnFwdReg(OUT_B, -POWER/2, OUT_REGMODE_SPEED);
  }
  else if (action == ACTION_B) {
    OnFwdReg(OUT_A, -POWER/2, OUT_REGMODE_SPEED);
    OnFwdReg(OUT_B,  POWER/2, OUT_REGMODE_SPEED);
  }
  else {
    OnFwdReg(OUT_A,  POWER, OUT_REGMODE_SPEED);
    OnFwdReg(OUT_B,  POWER, OUT_REGMODE_SPEED);
  }
  Wait(MOVE_WAIT_TIME);
}

float computeQ(int estado) {
  float Qs = Qmatrix[estado][0];
  for (int i=1; i<NUM_ACTIONS; ++i)
    Qs = max(Qs, Qmatrix[estado][i]);
  return Qs;
}

void waitForButton() {
  int button_state = BTNSTATE_NONE;
  while(button_state != BTNSTATE_SHORT_RELEASED_EV &&
	button_state != BTNSTATE_LONG_RELEASED_EV) {
    button_state = ButtonState(BTNCENTER);
  }
}

void calibrateLight() {
  TextOut(0, LCD_LINE1, "WHITE CALIBRATION...");
  waitForButton();
  WHITE = SensorRaw(IN_1);
  TextOut(0, LCD_LINE2, "WHITE = ");
  NumOut(50, LCD_LINE2, WHITE);
  Wait(500);
  PlayTone(880, 100);
  TextOut(0, LCD_LINE1, "BLACK CALIBRATION...");
  waitForButton();
  BLACK = SensorRaw(IN_1);
  TextOut(0, LCD_LINE3, "BLACK = ");
  NumOut(50, LCD_LINE3, BLACK);
  Wait(500);
  PlayTone(880, 100);
  Wait(2000);
  ClearScreen();
  if (WHITE > BLACK) {
    TextOut(0, LCD_LINE5, "CALIBRATION PROBLEM!!!!");
    PlayTone(440, 1000);
    waitForButton();
    abort();
  }
}

//PROGRAMA PRINCIPAL
task main(){
  int current_state, action, next_state;
  for (int i=0; i<NUM_STATES; i++)
    for (int j=0; j<NUM_ACTIONS; j++)
      Qmatrix[i][j]=0;
  
  // REWARDS PER STATE
  rewards[BB] = VERY_LITTLE_PENALTY;
  rewards[WB] = REWARD;
  rewards[BW] = VERY_LITTLE_PENALTY;
  rewards[WW] = PENALTY;

  SetSensorLight(IN_1);
  SetSensorLight(IN_2);
  SetSensorMode(IN_1, SENSOR_MODE_RAW);
  SetSensorMode(IN_2, SENSOR_MODE_RAW);
  
  
  calibrateLight();
  TextOut(0, LCD_LINE1, "GO                   ");
  waitForButton();
  ClearScreen();
  
  // FIRST MOVEMENT, GO STRAIGHT
  move(ACTION_C);
  current_state=leer();
  
  // MAIN LOOP
  while(TRUE){
    action = selectAction(current_state);
    move(action);
    next_state = leer();
    
    if (rewards[next_state] > 0.5)
      PlayTone(880, MOVE_WAIT_TIME/2);
    
    // APRENDER
    // ECUACION BASICA DEL Q-LEARNING
    // Q(s,a) = (1-alpha)*Q(s,a) + alpha*(R(s') + lambda*Q(s'))
    Qmatrix[current_state][action] = ((1 - ALPHA)*Qmatrix[current_state][action] +
				    ALPHA*(rewards[next_state] +
					  LAMBDA*computeQ(next_state)));
    
    // MOSTRAMOS ESTADO Y ESTADOSIGUIENTE EN PANTALLA DEL NXT
    NumOut(0, LCD_LINE6, current_state);
    NumOut(30, LCD_LINE6, action);
    NumOut(60, LCD_LINE6, next_state);
    
    // MOSTRAMOS QMATRIX Q
    MyNumOut(COL1, LCD_LINE1, Qmatrix[0][0]);
    MyNumOut(COL2, LCD_LINE1, Qmatrix[0][1]);
    MyNumOut(COL3, LCD_LINE1, Qmatrix[0][2]);
    MyNumOut(COL1, LCD_LINE2, Qmatrix[1][0]);
    MyNumOut(COL2, LCD_LINE2, Qmatrix[1][1]);
    MyNumOut(COL3, LCD_LINE2, Qmatrix[1][2]);
    MyNumOut(COL1, LCD_LINE3, Qmatrix[2][0]);
    MyNumOut(COL2, LCD_LINE3, Qmatrix[2][1]);
    MyNumOut(COL3, LCD_LINE3, Qmatrix[2][2]);
    MyNumOut(COL1, LCD_LINE4, Qmatrix[3][0]);
    MyNumOut(COL2, LCD_LINE4, Qmatrix[3][1]);
    MyNumOut(COL3, LCD_LINE4, Qmatrix[3][2]);
    
    // CAMBIO DE ESTADO
    current_state=next_state;
  }
}