/*
  Motors.cpp - Arduino Robotics Board
  V1 - Initial tests
  Tests for PID motor control class
  Author: Alejandro Pascual San Roman (bdh532)
  Sep 2024
*/

#include "Motors.h"
#include <ARB.h>

// CONSTRUCTOR

Motors::Motors(){
}

// STATIC MEMBER VARIABLE INITIALISATION

Motors::m_wheelDirection Motors::m_dirA = CW;
Motors::m_wheelDirection Motors::m_dirB = CCW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int Motors::m_stepsA = 0;
volatile int Motors::m_stepsB = 0;

// PUBLIC METHODS

void Motors::initialize(){
  m_setPinModes();
  m_attachInterrupts(); // Attaches interrupts
  m_initializeSerialRegisters(); // Initialize serial registers
}

void Motors::runMotors(){
//  m_readPidTunningSettings();
//  m_readSetpoints();
//  m_computePID(m_setpointA, m_setpointB, true);
  m_advanceRobot(10);
  m_rotateRobot(PI/4);
  m_advanceRobot(-10);
  m_rotateRobot(-PI/4);

  m_advanceRobot(5);
  m_rotateRobot(PI/2);
  m_advanceRobot(-5);
  m_rotateRobot(-PI/2);

  m_rotateRobot(PI);
  m_rotateRobot(-PI);

  Serial.print("Pose:");
    
  Serial.print(" [");
  Serial.print(m_pose[0]);
  Serial.print(", ");
  Serial.print(m_pose[1]);
  Serial.print(", ");
  Serial.print(m_pose[2]);
  Serial.println("]");
}

// PRIVATE MEMBERS

void Motors::m_setPinModes(){
  // Set relevant modes for pins
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
}

void Motors::m_attachInterrupts(){
  // Attach interrupts to motor encoder inputs so we don't miss any steps
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), m_ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), m_ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING
}

void Motors::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi
  
  // PID
  putRegister(REG_RECEIVE_KP_A, 0); // Receives proportional multiplier of motor A
  putRegister(REG_RECEIVE_KP_A_DEC, 0); // Receives proportional multiplier of motor A (decimal part)
  
  putRegister(REG_RECEIVE_KP_B, 0); // Receives proportional multiplier of motor B
  putRegister(REG_RECEIVE_KP_B_DEC, 0); // Receives proportional multiplier of motor B (decimal part)
  
  putRegister(REG_RECEIVE_KI_A, 0); // Receives integral multiplier of motor A
  putRegister(REG_RECEIVE_KI_A_DEC, 0); // Receives integral multiplier of motor A (decimal part)
  
  putRegister(REG_RECEIVE_KI_B, 0); // Receives integral multiplier of motor B
  putRegister(REG_RECEIVE_KI_B_DEC, 0); // Receives integral multiplier of motor B (decimal part)
  
  putRegister(REG_RECEIVE_KD_A, 0); // Receives derivative multiplier of motor A
  putRegister(REG_RECEIVE_KD_A_DEC, 0); // Receives derivative multiplier of motor A (decimal part)
  
  putRegister(REG_RECEIVE_KD_B, 0); // Receives derivative multiplier of motor B
  putRegister(REG_RECEIVE_KD_B_DEC, 0); // Receives derivative multiplier of motor B (decimal part)
  
  putRegister(REG_RECEIVE_SETPOINT_A, 0); // PID Setpoint of wheel A
  putRegister(REG_RECEIVE_SETPOINT_A_DEC, 0); // PID Setpoint of wheel A (decimal part)
  
  putRegister(REG_RECEIVE_SETPOINT_B, 0); // PID Setpoint of wheel B
  putRegister(REG_RECEIVE_SETPOINT_B_DEC, 0); // PID Setpoint of wheel B (decimal part)
  
  putRegister(REG_RECEIVE_TIMECHANGE, 0); // Millisecond delay between PID loop iterations
  putRegister(REG_RECEIVE_STEP_DISTANCE, 0); // Distance per step
  putRegister(REG_RECEIVE_DISTANCE_BETWEEN_WHEELS, 0); // Distance between wheels in centimetres
  putRegister(REG_RECEIVE_WHEEL_DIAMETER, 0); // Wheel diameter
  
  // POSE
  putRegister(REG_SEND_POSE_X, 0); // Robot pose x
  putRegister(REG_SEND_POSE_X_CM, 0); // Robot pose x (decimal part)
  
  putRegister(REG_SEND_POSE_Y, 0); // Robot pose y
  putRegister(REG_SEND_POSE_Y_CM, 0); // Robot pose y (decimal part)
  
  putRegister(REG_SEND_POSE_W, 0); // Robot pose w
  putRegister(REG_SEND_POSE_W_CM, 0); // Robot pose w (decimal part)
  
  // DRIVE DATA
  putRegister(REG_RECEIVE_STOP_SIGNAL, 0); // PID Stop signal
  putRegister(REG_RECEIVE_MSG_DRIVE, 0); // Receive drive control data
}

void Motors::m_readPidTunningSettings(){
  double kpAInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KP_A, REG_RECEIVE_KP_A_DEC);
  double kpBInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KP_B, REG_RECEIVE_KP_B_DEC);

  if(kpAInput != m_kpA){
    m_kpA = kpAInput;
  }

  if(kpBInput != m_kpB){
    m_kpB = kpBInput;
  }
  
  double kiAInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KI_A, REG_RECEIVE_KI_A_DEC);
  double kiBInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KI_B, REG_RECEIVE_KI_B_DEC);

  if(kiAInput != m_kiA){
    m_kiA = kiAInput;
  }

  if(kiBInput != m_kiB){
    m_kiB = kiBInput;
  }
  
  double kdAInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KD_A, REG_RECEIVE_KD_A_DEC);
  double kdBInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_KD_B, REG_RECEIVE_KD_B_DEC);

  if(kdAInput != m_kdA){
    m_kdA = kdAInput;
  }

  if(kdBInput != m_kdB){
    m_kdB = kdBInput;
  }
}

// Read setpoint settings from Pi
void Motors::m_readSetpoints(){
  double setpointAInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_SETPOINT_A, REG_RECEIVE_SETPOINT_A_DEC);
  double setpointBInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_SETPOINT_B, REG_RECEIVE_SETPOINT_B_DEC);

  if(setpointAInput != m_setpointA){
    m_setpointA = setpointAInput;
  }

  if(setpointBInput != m_setpointB){
    m_setpointB = setpointBInput;
  }
}

int Motors::m_computePID(double t_setpointA, double t_setpointB, bool stopAtGoal){
  int exitCode = 0;
  
  double stepsA_cm = m_stepsToCentimetres(m_stepsA);
  double stepsB_cm = m_stepsToCentimetres(m_stepsB);

  double errorA = t_setpointA - stepsA_cm; // m_stepsA_cm -> input
  double errorB = t_setpointB - stepsB_cm; // m_stepsB_cm -> input

  if(errorA > -m_goalMargin && errorA < m_goalMargin && stopAtGoal == true){
    m_setMotorSpeed(A, 0);
    m_resetErrors(A);
    exitCode += 1;
  }
  else {
    m_errorSumA += (errorA * m_timeChange);
    double errorDiffA = (errorA - m_lastErrorA) / m_timeChange;
    
    double proportionalA = m_kpA * errorA;
    double integralA = m_kiA * m_errorSumA;
    double derivativeA = m_kdA * errorDiffA;
  
    double m_speedA = proportionalA + integralA + derivativeA; // m_speedA -> output
  
    m_setMotorSpeed(A, m_speedA);
  
    m_lastErrorA = errorA;
  }

  if(errorB > -m_goalMargin && errorB < m_goalMargin && stopAtGoal == true){
    m_setMotorSpeed(B, 0);
    m_resetErrors(B);
    exitCode += 1;
  }
  else {
    m_errorSumB += (errorB * m_timeChange);
    double errorDiffB = (errorB - m_lastErrorB) / m_timeChange;
    
    double proportionalB = m_kpB * errorB;
    double integralB = m_kiB * m_errorSumB;
    double derivativeB = m_kdB * errorDiffB;
  
    double m_speedB = proportionalB + integralB + derivativeB; // m_speedA -> output
  
    m_setMotorSpeed(B, m_speedB);
  
    m_lastErrorB = errorB;
  }

  if(exitCode >= 2){
    m_calculateCurrentPose(stepsA_cm, stepsB_cm);
    m_stepsA = 0;
    m_stepsB = 0;
  }

  delay(m_timeChange);
  
  return exitCode;
}

// Converts encoder steps to centimetres
double Motors::m_stepsToCentimetres(int t_steps){
  double fullRotation = 298 * 6;
  double wheelRadius = m_wheelDiameter / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

void Motors::m_resetErrors(m_Motor t_motor){

  if(t_motor == A){
    m_errorSumA = 0;
    m_lastErrorA = 0;
  }
  else {
    m_errorSumB = 0;
    m_lastErrorB = 0;
  }
}

void Motors::m_setMotorSpeed(m_Motor t_motor, double t_speedPWM){

  t_speedPWM = t_speedPWM >= 255 ? 255 : (t_speedPWM <= -255 ? -255 : t_speedPWM);

  int pwmPin = t_motor == A ? MOTOR_PWMA : MOTOR_PWMB;
  int dirPin = t_motor == A ? MOTOR_DIRA : MOTOR_DIRB;
  
  m_wheelDirection wheelDirection = t_speedPWM < 0 ? (t_motor == A ? CCW : CW) : (t_motor == A ? CW : CCW);

  if (t_motor == A){
    m_dirA = wheelDirection;
  }
  else {
    m_dirB = wheelDirection;
  }

  digitalWrite(dirPin, wheelDirection);
  analogWrite(pwmPin, abs(t_speedPWM));
}

// Calculates the next pose based on calculated travelled distance of each wheel and stores it in m_pose[]
void Motors::m_calculateCurrentPose(double t_distanceA, double t_distanceB){
    // Calculations based on https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
    // double angleChange = (t_distanceA - t_distanceB) / m_distanceBetweenWheels;
    // double distanceChange = (t_distanceA - t_distanceB) / 2; // Since the movement is small, it assumes the circular distance is equal to the linear distance
    //    
    // m_pose[0] = m_pose[0] + distanceChange * cos(m_pose[2]+ (angleChange/2));
    // m_pose[1] = m_pose[1] + distanceChange * sin(m_pose[2]+ (angleChange/2));
    // m_pose[2] = m_pose[2] + angleChange;

    // With my own calculations. This is all geometrically correct, but may cause problems if the distances are similar(?), as the radius value would approach infinity.
    double radiusB = (t_distanceA != t_distanceB) ? (t_distanceB * m_distanceBetweenWheels)/(t_distanceA - t_distanceB) : 10000000000000000000; // Large value instead of infinity if distances are equal.
    double angleChange = t_distanceB/radiusB;
    double radius = radiusB + m_distanceBetweenWheels/2;
    double distanceChange = 2 * radius * sin(angleChange/2);

    m_pose[0] = m_pose[0] + distanceChange * cos(m_pose[2]+ (angleChange/2));
    m_pose[1] = m_pose[1] + distanceChange * sin(m_pose[2]+ (angleChange/2));
    m_pose[2] = m_pose[2] + angleChange;

    Serial.print("Pose:");
    
    Serial.print(" [");
    Serial.print(m_pose[0]);
    Serial.print(", ");
    Serial.print(m_pose[1]);
    Serial.print(", ");
    Serial.print(m_pose[2]);
    Serial.println("]");
}

// REGISTER DATA MANAGEMENT

// Sends large numbers to registers (up to + or - 14bit numbers)
void Motors::m_send16BitNumber(int t_register1, int t_register2, int t_number){
  int firstPart = t_number >> 7;
  int secondPart = t_number - (firstPart << 7);

  putRegister(t_register1, secondPart);
  putRegister(t_register2, firstPart);
}

// Reads large numbers from registers (up to + or - 16bit numbers)
double Motors::m_read16BitNumber(int t_register1, int t_register2){
  int firstPart = getRegister(t_register2);
  int secondPart = getRegister(t_register1);

  int result = (firstPart << 7) + secondPart; // Shifts first value by seven bits and add the second value
  return result;
}

// Sends decimal numbers to registers by splitting the whole from the fractionary part
void Motors::m_sendDecimalToRegisters(int t_register1, int t_register2, float t_number){
  int wholePart = (int)t_number;
  int fractPart = t_number*100 - wholePart*100;

  putRegister(t_register1, wholePart);
  putRegister(t_register2, fractPart);
}

double Motors::m_readDecimalNumberFromRegisters(int t_register1, int t_register2){
  double wholePart = (double)getRegister(t_register1);
  double fractPart = (double)getRegister(t_register2);
  
  double resultFrac = wholePart + fractPart/100;
  return resultFrac;
}

// READ REGISTER FUNCTIONS

void Motors::m_readDirectionInput(){
  // Get input data from register (WASD)
  int input = getRegister(REG_RECEIVE_MSG_DRIVE);

  // If the input has changed
  if(input != m_inputPrev){
    // Forward input
    if(input == 1){
      m_aimRobot(FORWARD);
      putRegister(REG_SEND_MSG_DRIVE, 1);
    }
    // Backward input
    else if (input == 2){
      m_aimRobot(BACKWARD);
      putRegister(REG_SEND_MSG_DRIVE, 2);
    }
    // Left input
    else if (input == 3){
      m_aimRobot(LEFT);
      putRegister(REG_SEND_MSG_DRIVE, 3);
    }
    // Right input
    else if (input == 4){
      m_aimRobot(RIGHT);
      putRegister(REG_SEND_MSG_DRIVE, 4);
    }
    // Anything else stops the motors
    else {
      putRegister(REG_SEND_MSG_DRIVE, 5);
    }
    
    m_inputPrev = input;
  }
}

void Motors::m_rotateRobot(double t_angle_radians){
  double distance = t_angle_radians * m_distanceBetweenWheels / 2;
  int exitCode = 0;

  while(exitCode < 2){
    exitCode = m_computePID(distance, -distance, true);
  }
}

void Motors::m_advanceRobot(double t_distance_cm){
  int exitCode = 0;
  
  while(exitCode < 2){
    exitCode = m_computePID(t_distance_cm, t_distance_cm, true);
  }
}

void Motors::m_moveRobot(m_robotDirection t_robotDirection, unsigned long t_duration){
  unsigned long currentTime = 0;
 
  while(currentTime < t_duration){

    m_aimRobot(t_robotDirection);
    
    currentTime += m_timeChange;
  }
}

void Motors::m_aimRobot(m_robotDirection t_robotDirection){
  double setpointA = (t_robotDirection == FORWARD || t_robotDirection == RIGHT) ? m_stepDistance_cm : -m_stepDistance_cm;
  double setpointB = (t_robotDirection == FORWARD || t_robotDirection == LEFT) ? m_stepDistance_cm : -m_stepDistance_cm;
  
  m_computePID(setpointA, setpointB, true);
}

/* 
  Since we know the direction the motors should be travelling in, we do not need
  the full quadrature output of the encoders, we only take one channel from each
  to save on pins.
  
  These ISRs will be called once per edge, either rising or falling, and will add
  to the number of steps if the motor is currently moving CW, and take away if CCW
*/

//ISR for reading MOTOR_ENCA
void Motors::m_ENCA_ISR(){
  if(m_dirA == CW){
    m_stepsA++;
  }
  else if(m_dirA == CCW){
    m_stepsA--;
  }
}

//ISR for reading MOTOR_ENCB
void Motors::m_ENCB_ISR(){
  if(m_dirB == CCW){
    m_stepsB++;
  }
  else if(m_dirB == CW){
    m_stepsB--;
  }
}
