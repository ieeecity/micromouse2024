/** DO NOT CHANGE THE FOLLOWING DEFINITONS - From UKMARS MazeRunner GitHub **/
/** =============================================================================================================== **/
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) fast_write_pin(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif


#include "Queue.h"
/** =============================================================================================================== **/

/** DEFINE OUR PINS AND WHICH COMPONENTS THEY ARE CONNECTED TO **/
/** _______________________________________________________________________________________________________________ **/
const int EMITTERS = 12; // EMITTERS
const int RED_LED = 13; // RED LED AT H BRIDGE

const int INDICATOR_LED_R = 6; // INDICATOR LED RIGHT 
const int INDICATOR_LED_L = 11; // INDICATOR LED LEFT

const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) 

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

// Phototransistors
const int RIGHT_SENSOR = A0;
const int LEFT_SENSOR = A2;
const int MIDDLE_SENSOR = A1;

// 4 Way switch and push button
const int DIP_SWITCH = A6; 
/** _______________________________________________________________________________________________________________ **/

/* GLOBAL VARIABLES */
volatile int rightEncoderPos = 0; // Counts for right encoder ticks
volatile int leftEncoderPos = 0; // Counts for left encoder ticks

// Variables to help us with our PID
int prevTime = 0;
int prevError_l;
int prevError_r;
int errorIntegral_l;
int errorIntegral_r;

// Flag variable to indicate whether the switch is on or off
bool switchOn = false;

// Variables to keep track of our sensors
int sensorThreshold = 22;

// Variables to keep track of where we are in the maze with coordinates
Cell &START = maze[0][0];
Cell &GOAL = maze[5][5];
Cell currPos;
String prevHeading = "NORTH"; // can be NORTH, EAST, SOUTH, WEST, initialise to NORTH 

// Variables for setting our target moves - this will be tuned to everyone differently 
int forwardTarget = 99; // Change these to your own values for tuning 
int rotateTarget = 44; // Change these to your own values for tuning
int forwardPID[3] = {1, 2, 0}; // kp ki kd for going forwards
int rotatePID[3] = {3, 2, 2}; // kp ki kd for rotating

int FWD_COUNT = 0;

/** _______________________________________________________________________________________________________________ **/

/** INTERRUPT SERVICE ROUTINES FOR HANDLING ENCODER COUNTING USING STATE TABLE METHOD **/
void readEncoderLeft() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_L_B) << 1) | fast_read_pin(ENCODER_L_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  // direction based on prev state
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      leftEncoderPos--;
      break;
    case 0b0010:
    case 0b1100:
    case 0b0101:
    case 0b1011:
      leftEncoderPos++;
      break;

    default:
      break;
  }
  prevState = currState;
}

void readEncoderRight() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_R_B) << 1) | fast_read_pin(ENCODER_R_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0100:
    case 0b1010:
    case 0b0111:
    case 0b1001:
      rightEncoderPos++;
      break;
    case 0b1000:
    case 0b0110:
    case 0b1101:
    case 0b0011:
      rightEncoderPos--;
      break;

    default:
      break;
  }
  prevState = currState;
}

/** Function to set motor speed and direction for BOTH motors
    @params dir - can either be HIGH or LOW for clockwise / counter clockwise rotation
    @params speed - analogWrite() value between 0-255
**/
//==============================================================================================
void setMotor_r(int dir, int speed){
  analogWrite(SPEED_MOTOR_R, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_R, LOW);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_R, HIGH);
  } else{
    analogWrite(SPEED_MOTOR_R, 0);
  }
}

void setMotor_l(int dir, int speed){
  analogWrite(SPEED_MOTOR_L, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_L, HIGH);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_L, LOW);
  } else{
    analogWrite(SPEED_MOTOR_L, 0);
  }
}
//==============================================================================================

/** Function to make the robot travel for a certain amount of encoder ticks, calls upon setMotors at end
    @params dir - setPoint: The target value for how far we want to go (in encoder ticks)
    @params speed - analogWrite() value between 0-255
    @params kp - proportional gain, this is the main one you should be changing
    @params ki - intergral gain, use this for steady state errors
    @params kd - derivative gain, use this for overshoot and oscillation handling 
**/
void motorPID_r(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime)) / 1.0e6; // time difference between ticks in seconds
  prevTime = currentTime; // update prevTime each loop 
  
  int error = setPoint - rightEncoderPos;
  int errorDerivative_r = (error - prevError_r) / deltaT;
  errorIntegral_r = errorIntegral_r + error*deltaT;

  float u = kp*error + ki*errorIntegral_r + kd*errorDerivative_r; 

  float speed = fabs(u); // Set a top speed
  if(speed > 150){
    speed = 150;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotor_r(dir, speed);
  prevError_r = 0;
}

void motorPID_l(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime)) / 1.0e6; // time difference between ticks in seconds
  prevTime = currentTime; // update prevTime each loop 
  
  int error = setPoint - leftEncoderPos;
  int errorDerivative_l = (error - prevError_l) / deltaT;
  errorIntegral_l = errorIntegral_l + error*deltaT;

  float u = kp*error + ki*errorIntegral_l + kd*errorDerivative_l; 

  float speed = fabs(u); // Set a top speed
  if(speed > 150){
    speed = 150;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotor_l(dir, speed);
  prevError_l = 0;
}

// Reset our encoder ticks
void resetCount(){
  rightEncoderPos = 0;
  leftEncoderPos = 0;
  setMotor_r(0, 0);
  setMotor_l(0, 0);
}

// Helper functions when moving
void goForward(){
  motorPID_r(forwardTarget, forwardPID[0]-0.0588, forwardPID[1], forwardPID[2]);
  motorPID_l(forwardTarget, forwardPID[0]+0.005, forwardPID[1], forwardPID[2]);
  while (leftEncoderPos < forwardTarget && rightEncoderPos < forwardTarget) {
    delay(30); 
  }
}

void turnRight(){
  motorPID_r(-rotateTarget, rotatePID[0], rotatePID[1], rotatePID[2]);
  motorPID_l(rotateTarget, rotatePID[0], rotatePID[1], rotatePID[2]);
  while (leftEncoderPos < rotateTarget && rightEncoderPos < rotateTarget) {
    delay(30); 
  }
}

void turnLeft(){
  motorPID_r(rotateTarget+5, rotatePID[0], rotatePID[1], rotatePID[2]);
  motorPID_l(-rotateTarget+5, rotatePID[0], rotatePID[1], rotatePID[2]);
  while (leftEncoderPos < rotateTarget && rightEncoderPos < rotateTarget) {
    delay(30); 
  }
}

void turnAround(){
  motorPID_r(rotateTarget*2, rotatePID[0], rotatePID[1], rotatePID[2]);
  motorPID_l(-rotateTarget*2, rotatePID[0], rotatePID[1], rotatePID[2]);
  while (leftEncoderPos < rotateTarget && rightEncoderPos < rotateTarget) {
    delay(30); 
  }
}

/** Floodfill function to update the cell values based on found walls, check slides lecture 2
    @params maze - takes and modifies the ACTUAL maze by reference
**/ 
void floodfill(struct Cell (&maze)[8][8]){

  // Set all the weights to a "blank state of -1"
  for(int i = 0; i < 8; ++i){
    for(int j = 0; j < 8; ++j){
      maze[i][j].weight = -1;
    }
  }

  // Set goal cell to 0 and add to Queue
  maze[GOAL.y][GOAL.x].weight = 0;
  enqueue(maze[GOAL.y][GOAL.x]);

  while(Queue.size() > 0){
    Cell consider = front(); // Look at the front of the queue and consider that cell
    dequeue();
    
    if(consider.y < 8){
      Cell &northern = maze[consider.y+1][consider.x];
      if(northern.weight < 0 && !(consider.walls[2]) && !(northern.walls[3])){ // If the current cell's north is blank and accessible from both ends
          northern.weight = consider.weight+1;
          enqueue(northern);
      }
    }

    if(consider.x < 8){
      Cell &eastern = maze[consider.y][consider.x+1];
      if(eastern.weight < 0 && !(consider.walls[0]) && !(eastern.walls[1])){ // If the current cell's east is blank and accessible from both ends
          eastern.weight = consider.weight+1;
          enqueue(eastern);
      }
    }

    if(consider.y >= 0){
      Cell &southern = maze[consider.y-1][consider.x];
       if(southern.weight < 0 && !(consider.walls[3]) && !(southern.walls[2])){ // If the current cell's south is blank and accessible from both ends
          southern.weight = consider.weight+1;
          enqueue(southern);
      }
    }

    if(consider.x >= 0){
      Cell &western = maze[consider.y][consider.x-1];
      if(western.weight < 0 && !(consider.walls[1]) && !(western.walls[0])){ // If the current cell's south is blank and accessible from both ends
          western.weight = consider.weight+1;
          enqueue(western);
      }
    }    
  }
}


void setup() {
  Serial.begin(9600);
  
  pinMode(EMITTERS, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(INDICATOR_LED_R, OUTPUT);
  pinMode(INDICATOR_LED_L, OUTPUT);

  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);

  pinMode(DIP_SWITCH, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), readEncoderRight, CHANGE);

  
  floodfill(maze); // Initialise the maze before we start the loop
  currPos = START; // Set out current position to the starting cell
  printMazeWeights(maze); 

  Serial.println("GOAL: ");
  Serial.print(GOAL.x);
  Serial.print(",");
  Serial.print(GOAL.y);
  Serial.print(" GOAL WEIGHT: ");
  Serial.print(GOAL.weight);
  Serial.println("\n");

  Serial.println("START: ");
  Serial.print(currPos.x);
  Serial.print(",");
  Serial.print(currPos.y);
  Serial.print(" CURRENT WEIGHT: ");
  Serial.print(currPos.weight);
  Serial.println("");
}


void loop() {

  int dipSwitch = analogRead(DIP_SWITCH);
  if(dipSwitch > 1000){
    switchOn = true;
  }

  if(switchOn){
    delay(5000);
    while(currPos.x != GOAL.x || currPos.y != GOAL.y){ // Do this until we get to the goal
      digitalWrite(EMITTERS, HIGH);

      // If our sensors detect new walls, update our maze
      if(analogRead(RIGHT_SENSOR) >  sensorThreshold){
        maze[currPos.y][currPos.x].walls[0] = true;
        maze[currPos.y][currPos.x+1].walls[1] = true;
      }
      if(analogRead(MIDDLE_SENSOR) >  sensorThreshold){
        maze[currPos.y][currPos.x].walls[2] = true;
        maze[currPos.y-1][currPos.x].walls[3] = true;
      }
      if(analogRead(LEFT_SENSOR) >  sensorThreshold){
        maze[currPos.y][currPos.x].walls[1] = true;
        maze[currPos.y][currPos.x+1].walls[0] = true;
      }
    
    // Check each cardinal direction's cell to see if its the lowest to move to
    if(currPos.y < 8){
      Cell &northern = maze[currPos.y+1][currPos.x];
      if(northern.weight < currPos.weight && !(currPos.walls[2])){ // If northern cell is smaller and accessible then go to it
        if(prevHeading == "NORTH"){
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "EAST"){
          turnLeft();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "SOUTH"){
          turnAround();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "WEST"){
          turnRight();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        }

        //Serial.println("GOING NORTH FROM ");
        Serial.print("(");
        Serial.print(currPos.x);
        Serial.print(",");
        Serial.print(currPos.y);
        Serial.print(") ");
        Serial.print("TO ");
        Serial.print("(");
        Serial.print(northern.x);
        Serial.print(",");
        Serial.print(northern.y);
        Serial.print(")\n");

        currPos = northern;
        prevHeading = "NORTH"; // Update relative heading 
        delay(500);
      }
    }
      
    if(currPos.x < 8){
      Cell &eastern = maze[currPos.y][currPos.x+1];
      if(eastern.weight < currPos.weight && !(currPos.walls[0])){ // If eastern cell is smaller and accessible then go to it
        if(prevHeading == "NORTH"){
          turnRight();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "EAST"){
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "SOUTH"){
          turnLeft();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "WEST"){
          turnAround();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        }

        //Serial.println("GOING EAST FROM ");
        Serial.print("(");
        Serial.print(currPos.x);
        Serial.print(",");
        Serial.print(currPos.y);
        Serial.print(") ");
        Serial.print("TO ");
        Serial.print("(");
        Serial.print(eastern.x);
        Serial.print(",");
        Serial.print(eastern.y);
        Serial.print(")\n");

        currPos = eastern;
        prevHeading = "EAST"; // Update relative heading 
        delay(500);
      }
    }
      
    if(currPos.y >= 0){
      Cell &southern = maze[currPos.y-1][currPos.x];
      if(southern.weight < currPos.weight && !(currPos.walls[3])){ // If southern cell is smaller and accessible then go to it
        if(prevHeading == "NORTH"){
          turnAround();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "EAST"){
          turnRight();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "SOUTH"){
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "WEST"){
          turnLeft();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        }

        //Serial.println("GOING SOUTH FROM ");
        Serial.print("(");
        Serial.print(currPos.x);
        Serial.print(",");
        Serial.print(currPos.y);
        Serial.print(") ");
        Serial.print("TO ");
        Serial.print("(");
        Serial.print(southern.x);
        Serial.print(",");
        Serial.print(southern.y);
        Serial.print(")\n");

        currPos = southern;
        prevHeading = "SOUTH"; // Update relative heading 
        delay(500);
      }
    }

    if(currPos.x >= 0){
      Cell &western = maze[currPos.y][currPos.x-1];
      if(western.weight < currPos.weight && !(currPos.walls[1])){ // If western cell is smaller and accessible then go to it
        if(prevHeading == "NORTH"){
          turnLeft();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "EAST"){
          turnAround();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "SOUTH"){
          turnRight();
          resetCount();
          delay(200);
          goForward();
          resetCount();
          delay(200);
        } else if(prevHeading == "WEST"){
          goForward();
          resetCount();
          delay(200);
        }

        //Serial.println("GOING WEST FROM ");
        Serial.print("(");
        Serial.print(currPos.x);
        Serial.print(",");
        Serial.print(currPos.y);
        Serial.print(") ");
        Serial.print("TO ");
        Serial.print("(");
        Serial.print(western.x);
        Serial.print(",");
        Serial.print(western.y);
        Serial.print(")\n");

        currPos = western;
        prevHeading = "WEST"; // Update relative heading 
        delay(500);
      }
    }
      // After we have selected, moved to a cell and updated our position lets flood the maze again and see what the current state is
      delay(50);
      Serial.println("");
      floodfill(maze); // reflood the maze in case new walls found
      printMazeWeights(maze); // see what the maze looks like now that new walls detected
      delay(50);
    }
      Serial.println("DONE!");
      Serial.println("===========");
      Serial.println("Finished Maze: ");
      Serial.print("(");
      Serial.print(currPos.x);
      Serial.print(",");
      Serial.print(currPos.y);
      Serial.print(") \n");
      digitalWrite(EMITTERS, LOW);
  }
    switchOn = false;
    setMotor_l(0,0); // Stop the mouse
    setMotor_r(0,0);
}
