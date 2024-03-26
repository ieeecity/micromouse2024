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

/** =============================================================================================================== **/

/** DEFINE OUR PINS AND WHICH COMPONENTS THEY ARE CONNECTED TO **/
/** _______________________________________________________________________________________________________________ **/
const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) 

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

// 4 Way switch and push button
const int DIP_SWITCH = A6; 
/** _______________________________________________________________________________________________________________ **/

/* GLOBAL VARIABLES */

volatile int rightEncoderPos = 0; // Counts for right encoder ticks
volatile int leftEncoderPos = 0; // Counts for left encoder ticks

// Variables to help us with our PID
int prevTime = 0;
int prevError;
int errorIntegral;
bool switchOn = false;

void setup() {
  Serial.begin(9600);
  
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
}

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
      leftEncoderPos++;
      break;
    case 0b0010:
    case 0b1100:
    case 0b0101:
    case 0b1011:
      leftEncoderPos--;
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
void setMotors(int dir, int speed){
  analogWrite(SPEED_MOTOR_L, speed);
  analogWrite(SPEED_MOTOR_R, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_L, HIGH);
    fast_write_pin(DIR_MOTOR_R, LOW);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_L, LOW);
    fast_write_pin(DIR_MOTOR_R, HIGH);
  } else{
    analogWrite(SPEED_MOTOR_L, 0);
    analogWrite(SPEED_MOTOR_R, 0);
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
void motorPID(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime)) / 1.0e6; // time difference between ticks in seconds
  prevTime = currentTime; // update prevTime each loop 
  
  int error = setPoint - rightEncoderPos;
  int errorDerivative = (error - prevError) / deltaT;
  errorIntegral = errorIntegral + error*deltaT;

  float u = kp*error + ki*errorIntegral + kd*errorDerivative; 

  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotors(dir, speed);
  prevError = 0;
}
//==============================================================================================
//==============================================================================================
// YOUR HOMEWORK ASSIGNMENT: Create a function to convert from encoder ticks to centimeters!
int tickConvertToCm(int encoderTicks){
  // Your code here 
  return 0;
}
//==============================================================================================

void loop() {
  // Run continuous code here
}
