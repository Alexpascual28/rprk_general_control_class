/*
	differentialPidControl.ino - Arduino Robotics Board
  V1 - Initial tests
	Tests for PID motor control
  Author: Alejandro Pascual San Roman (bdh532)
	Aug 2024
*/

#include <ARB.h>

typedef enum {CW,CCW} m_wheelDirection; // CW = 0, CCW = 1
typedef enum {A,B} m_Motor; // A = 0, B = 1
typedef enum {FORWARD, RIGHT, BACKWARD, LEFT} m_robotDirection;

// STATIC MEMBER VARIABLE INITIALISATION

m_wheelDirection m_dirA = CW;
m_wheelDirection m_dirB = CCW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int m_stepsA = 0;
volatile int m_stepsB = 0;

// PID

bool m_runPID = true;

double m_errorSumA = 0;
double m_errorSumB = 0;
double m_lastErrorA = 0;
double m_lastErrorB = 0;

// Motor A tunings

const double m_kpA = 0.8;
const double m_kiA = 0.1;
const double m_kdA = 0.5;

// Motor B tunings

const double m_kpB = 0.8;
const double m_kiB = 0.1;
const double m_kdB = 0.5;

const unsigned long m_timeChange = 1; // 1 millisecond delay per PID compute
const double m_stepDistance_cm = 5; // Distance per step

// ODOMETRY

const double m_distanceBetweenWheels = 20; // In cm
const double m_wheelDiameter = 4.75;

double m_pose[3] = {0,0,0};

void setup() {
	ARBSetup(true); // Setup everything required by the board and enable serial comms
 
	// Set relevant modes for pins
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), m_ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), m_ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING
  
	Serial.begin(9600); // Start serial for debugging
}

// This example loop runs each robot component sequencially
void loop() {

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

  // Call the serialUpdate function at least once per loop
  serialUpdate();
}

void m_rotateRobot(double t_angle_radians){
  double distance = t_angle_radians * m_distanceBetweenWheels / 2;
  int exitCode = 0;

  while(exitCode < 2){
    exitCode = m_computePID(distance, -distance, true);
  }
}

void m_advanceRobot(double t_distance_cm){
  int exitCode = 0;
  
  while(exitCode < 2){
    exitCode = m_computePID(t_distance_cm, t_distance_cm, true);
  }
}

void m_moveRobot(m_robotDirection t_robotDirection, unsigned long t_duration){
  unsigned long currentTime = 0;
 
  while(currentTime < t_duration){

    m_aimRobot(t_robotDirection);
    
    currentTime += m_timeChange;
  }
}

void m_aimRobot(m_robotDirection t_robotDirection){
  double setpointA = (t_robotDirection == FORWARD || t_robotDirection == RIGHT) ? m_stepDistance_cm : -m_stepDistance_cm;
  double setpointB = (t_robotDirection == FORWARD || t_robotDirection == LEFT) ? m_stepDistance_cm : -m_stepDistance_cm;
  
  m_computePID(setpointA, setpointB, true);
}

int m_computePID(double t_setpointA, double t_setpointB, bool stopAtGoal){
  int exitCode = 0;
  
  double stepsA_cm = m_stepsToCentimetres(m_stepsA);
  double stepsB_cm = m_stepsToCentimetres(m_stepsB);

  double errorA = t_setpointA - stepsA_cm; // m_stepsA_cm -> input
  double errorB = t_setpointB - stepsB_cm; // m_stepsB_cm -> input

  if(errorA > -0.01 && errorA < 0.01 && stopAtGoal == true){
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

  if(errorB > -0.01 && errorB < 0.01 && stopAtGoal == true){
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
double m_stepsToCentimetres(int t_steps){
  double fullRotation = 298 * 6;
  double wheelRadius = m_wheelDiameter / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

void m_resetErrors(m_Motor t_motor){

  if(t_motor == A){
    m_errorSumA = 0;
    m_lastErrorA = 0;
  }
  else {
    m_errorSumB = 0;
    m_lastErrorB = 0;
  }
}

void m_setMotorSpeed(m_Motor t_motor, double t_speedPWM){

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
void m_calculateCurrentPose(double t_distanceA, double t_distanceB){
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

/* 
  Since we know the direction the motors should be travelling in, we do not need
  the full quadrature output of the encoders, we only take one channel from each
  to save on pins.
  
  These ISRs will be called once per edge, either rising or falling, and will add
  to the number of steps if the motor is currently moving CW, and take away if CCW
*/

//ISR for reading MOTOR_ENCA
void m_ENCA_ISR(){
  if(m_dirA == CW){
    m_stepsA++;
  }
  else if(m_dirA == CCW){
    m_stepsA--;
  }
}

//ISR for reading MOTOR_ENCB
void m_ENCB_ISR(){
  if(m_dirB == CCW){
    m_stepsB++;
  }
  else if(m_dirB == CW){
    m_stepsB--;
  }
}
