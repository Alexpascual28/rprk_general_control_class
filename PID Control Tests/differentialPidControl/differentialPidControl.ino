/*
	differentialPidControl.ino - Arduino Robotics Board
  V1 - Initial tests
	Tests for PID motor control
  Author: Alejandro Pascual San Roman (bdh532)
	Feb 2024
*/

#include <ARB.h>

typedef enum {CW,CCW} m_Direction; // CW = 0, CCW = 1
typedef enum {A,B} m_Motor; // A = 0, B = 1

// STATIC MEMBER VARIABLE INITIALISATION

m_Direction m_dirA = CW;
m_Direction m_dirB = CCW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int m_stepsA = 0;
volatile int m_stepsB = 0;

// PID

double m_errorSumA = 0;
double m_errorSumB = 0;
double m_lastErrorA = 0;
double m_lastErrorB = 0;

double m_kpA, m_kiA, m_kdA; // Motor A tunings
double m_kpB, m_kiB, m_kdB; // Motor B tunings

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

  m_kpA = 30;
  m_kiA = 0.8;
  m_kdA = 80;

  m_kpB = 30;
  m_kiB = 0.8;
  m_kdB = 80;
  
	Serial.begin(9600); // Start serial for debugging
}

// This example loop runs each robot component sequencially
void loop() {

  m_computePID(-10, 10, 1);
  
  // Call the serialUpdate function at least once per loop
  serialUpdate();
  
}

void m_computePID(double t_setpointA, double t_setpointB, unsigned long timeChange){
  
  double stepsA_cm = m_stepsToCentimetres(m_stepsA);
  double stepsB_cm = m_stepsToCentimetres(m_stepsB);

  double errorA = t_setpointA - stepsA_cm; // m_stepsA_cm -> input
  double errorB = t_setpointB - stepsB_cm; // m_stepsB_cm -> input

  if(errorA > -0.01 && errorA < 0.01){
    setMotorSpeed(A, 0);
    resetErrors(A);
  }
  else {
    m_errorSumA += (errorA * timeChange);
    double errorDiffA = (errorA - m_lastErrorA) / timeChange;
    
    double proportionalA = m_kpA * errorA;
    double integralA = m_kiA * m_errorSumA;
    double derivativeA = m_kdA * errorDiffA;
  
    double m_speedA = proportionalA + integralA + derivativeA; // m_speedA -> output
  
    setMotorSpeed(A, m_speedA);
  
    m_lastErrorA = errorA;
  }

  if(errorB > -0.01 && errorB < 0.01){
    setMotorSpeed(B, 0);
    resetErrors(B);
  }
  else {
    m_errorSumB += (errorB * timeChange);
    double errorDiffB = (errorB - m_lastErrorB) / timeChange;
    
    double proportionalB = m_kpB * errorB;
    double integralB = m_kiB * m_errorSumB;
    double derivativeB = m_kdB * errorDiffB;
  
    double m_speedB = proportionalB + integralB + derivativeB; // m_speedA -> output
  
    setMotorSpeed(B, m_speedB);
  
    m_lastErrorB = errorB;
  }

  delay(timeChange);
}

// Converts encoder steps to centimetres
double m_stepsToCentimetres(int t_steps){
  double fullRotation = 298 * 6;
  double wheelRadius = 4.75 / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

void resetErrors(m_Motor t_motor){

  if(t_motor == A){
    m_errorSumA = 0;
    m_lastErrorA = 0;
  }
  else {
    m_errorSumB = 0;
    m_lastErrorB = 0;
  }
  
}

void setMotorSpeed(m_Motor t_motor, double t_speedPWM){

  t_speedPWM = t_speedPWM >= 255 ? 255 : (t_speedPWM <= -255 ? -255 : t_speedPWM);

  int pwmPin = t_motor == A ? MOTOR_PWMA : MOTOR_PWMB;
  int dirPin = t_motor == A ? MOTOR_DIRA : MOTOR_DIRB;
  
  m_Direction wheelDirection = t_speedPWM < 0 ? (t_motor == A ? CCW : CW) : (t_motor == A ? CW : CCW);

  if (t_motor == A){
    m_dirA = wheelDirection;
  }
  else {
    m_dirB = wheelDirection;
  }

  digitalWrite(dirPin, wheelDirection);
  analogWrite(pwmPin, abs(t_speedPWM));
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
