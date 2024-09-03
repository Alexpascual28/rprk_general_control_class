#include "Motors.h"
#include <ARB.h>
#include <Wire.h>
#include <PID_v1.h>

// CONSTRUCTOR

Motors::Motors(){
}

// STATIC MEMBER VARIABLE INITIALISATION

Motors::m_Direction Motors::m_dirA = CW;
Motors::m_Direction Motors::m_dirB = CCW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int Motors::m_stepsA = 0;
volatile int Motors::m_stepsB = 0;

// These values will not be reset unless a signal is received
volatile int Motors::m_absoluteStepsA = 0;
volatile int Motors::m_absoluteStepsB = 0;

// PUBLIC METHODS

void Motors::initialize(){
  m_setPinModes();
  m_initializeComponents(); // Initializes values
  m_attachInterrupts(); // Attaches interrupts
  m_initializeSerialRegisters(); // Initialize serial registers
}

void Motors::runMotors(){
  m_readControlModeRegister();

  if(m_pidControlMode == true){
    m_readPidTunningSettings();
    m_runPidControl();
  }
  else {
    // Wheels
    m_readWheelDirections();
    m_readPWMSignals();

    // General movement
    m_readSpeedLevelValue();
    m_readDirectionInput();

    // Encoders
    m_setAbsoluteEncoderSteps();
  }
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

void Motors::m_initializeComponents(){
  // Set motors off by default
  m_moveForward();
  m_stopRobot();

  m_resetEncoders();
}

void Motors::m_attachInterrupts(){
  // Attach interrupts to motor encoder inputs so we don't miss any steps
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), m_ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), m_ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING
}

void Motors::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi
  // Motors
  putRegister(REG_RECEIVE_DIR_MOTOR_A, 0); // Motor A receive direction
  putRegister(REG_RECEIVE_PWM_MOTOR_A, 0); // Motor A receive PWM
  putRegister(REG_SEND_DATA_MOTOR_A, 0); // Motor A send data (current direction and speed)
  
  putRegister(REG_RECEIVE_DIR_MOTOR_B, 1); // Motor B receive direction
  putRegister(REG_RECEIVE_PWM_MOTOR_B, 0); // Motor B receive PWM
  putRegister(REG_SEND_DATA_MOTOR_B, 0); // Motor B send data (current direction and speed)

  putRegister(REG_SEND_DATA_ENCODER_A_1, 0); // Encoder A send data
  putRegister(REG_SEND_DATA_ENCODER_A_2, 0); // Encoder A send data
  putRegister(REG_SEND_DATA_ENCODER_B_1, 0); // Encoder B send data
  putRegister(REG_SEND_DATA_ENCODER_B_2, 0); // Encoder B send data

  putRegister(REG_RECEIVE_SPEED_DATA, 0); // Receive speed data
  putRegister(REG_RECEIVE_MSG_DRIVE, 0);// Receive drive control data (forward, backward, left, right)
  putRegister(REG_SEND_MSG_DRIVE, 0); // Send drive control data (current state)

  putRegister(REG_SEND_DISTANCE_A, 0); // Calculated distance in wheel A
  putRegister(REG_SEND_DISTANCE_A_DEC, 0); // Calculated distance in wheel A (decimal part)
  
  putRegister(REG_SEND_DISTANCE_B, 0); // Calculated distance in wheel B
  putRegister(REG_SEND_DISTANCE_B_DEC, 0); // Calculated distance in wheel B (decimal part)
  
  putRegister(REG_SEND_SPEED_A, 0); // Calculated speed of wheel A
  putRegister(REG_SEND_SPEED_A_DEC, 0); // Calculated speed of wheel A (decimal part)
  
  putRegister(REG_SEND_SPEED_B, 0); // Calculated speed of wheel B
  putRegister(REG_SEND_SPEED_B_DEC, 0); // Calculated speed of wheel B (decimal part)

  putRegister(REG_RECEIVE_RESET_ENCODER_A, 0);
  putRegister(REG_RECEIVE_RESET_ENCODER_B, 0);

  putRegister(REG_RECEIVE_CONTROL_MODE, 0); // Receives control mode input

  putRegister(REG_RECEIVE_KP_A, 0); // Receives proportional multiplier of motor A
  putRegister(REG_RECEIVE_KP_B, 0); // Receives proportional multiplier of motor B

  putRegister(REG_RECEIVE_KI_A, 0); // Receives integral multiplier of motor A
  putRegister(REG_RECEIVE_KI_B, 0); // Receives integral multiplier of motor B

  putRegister(REG_RECEIVE_KD_A, 0); // Receives derivative multiplier of motor A
  putRegister(REG_RECEIVE_KD_B, 0); // Receives derivative multiplier of motor B

  putRegister(REG_RECEIVE_SETPOINT_A, 0); // PID Setpoint of wheel A
  putRegister(REG_RECEIVE_SETPOINT_A_DEC, 0); // PID Setpoint of wheel A (decimal part)

  putRegister(REG_RECEIVE_SETPOINT_B, 0); // PID Setpoint of wheel B
  putRegister(REG_RECEIVE_SETPOINT_B_DEC, 0); // PID Setpoint of wheel B (decimal part)
}

void Motors::m_readControlModeRegister(){
  int controlModeInput = getRegister(REG_RECEIVE_CONTROL_MODE);

  if(controlModeInput != m_controlModeInputPrev){
    if(controlModeInput == 0){
      m_pidControlMode = false;
    }
    else {
      m_pidControlMode = true;
    }
    
    m_controlModeInputPrev = controlModeInput;
  }
}

void Motors::m_readPidTunningSettings(){
  double kpAInput = (double)getRegister(REG_RECEIVE_KP_A) / 100;
  double kpBInput = (double)getRegister(REG_RECEIVE_KP_B) / 100;

  if(kpAInput != m_kpA){
    m_kpA = kpAInput;
  }

  if(kpBInput != m_kpB){
    m_kpB = kpBInput;
  }
  
  double kiAInput = (double)getRegister(REG_RECEIVE_KI_A) / 100;
  double kiBInput = (double)getRegister(REG_RECEIVE_KI_B) / 100;

  if(kiAInput != m_kiA){
    m_kiA = kiAInput;
  }

  if(kiBInput != m_kiB){
    m_kiB = kiBInput;
  }
  
  double kdAInput = (double)getRegister(REG_RECEIVE_KD_A) / 100;
  double kdBInput = (double)getRegister(REG_RECEIVE_KD_B) / 100;

  if(kdAInput != m_kdA){
    m_kdA = kdAInput;
  }

  if(kdBInput != m_kdB){
    m_kdB = kdBInput;
  }
  
  double setpointAInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_SETPOINT_A, REG_RECEIVE_SETPOINT_A_DEC);
  double setpointBInput = m_readDecimalNumberFromRegisters(REG_RECEIVE_SETPOINT_B, REG_RECEIVE_SETPOINT_B_DEC);

  if(setpointAInput != m_setpointA){
    m_setpointA = setpointAInput;
    m_stepsA = 0;
    m_lastErrorA = 0;
    m_lastTimeA = millis();
  }

  if(setpointBInput != m_setpointB){
    m_setpointB = setpointBInput;
    m_stepsB = 0;
    m_lastErrorB = 0;
    m_lastTimeB = millis();
  }
}

// Probably change this functions name
void Motors::m_runPidControl(){
  m_convertStepsToCM();

  if(m_stepsA_cm != m_setpointA){
    m_computePID(A);
    m_motorSetSpeedCM(A, m_speedA);
  }

  if(m_stepsB_cm != m_setpointB){
    m_computePID(B);
    m_motorSetSpeedCM(B, m_speedB);
  }
}

void Motors::m_convertStepsToCM(){
  if(m_stepsA != m_stepsAprev) {
    m_stepsA_cm = m_stepsToCentimetres(m_stepsA);
    m_stepsAprev = m_stepsA;
  }

  if(m_stepsB != m_stepsBprev) {
    m_stepsB_cm = m_stepsToCentimetres(m_stepsB);
    m_stepsBprev = m_stepsB;
  }
}

void Motors::m_computePID(m_Motor t_motor){
  if(t_motor == A){
    unsigned long now = millis();
    double timeChange = (double)(now - m_lastTimeA);
  
    double error = m_setpointA - m_stepsA_cm; // m_stepsA_cm -> input
    m_errorSumA += (error * timeChange);
    double errorDiff = (error - m_lastErrorA) / timeChange;
    
    double proportional = m_kpA * error;
    double integral = m_kiA * m_errorSumA;
    double derivative = m_kdA * errorDiff;
  
    m_speedA = proportional + integral + derivative; // m_speedA -> output
    Serial.println(m_speedA);
    
    m_sendDecimalToRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC, m_speedA);
    m_sendDecimalToRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC, error);
  
    m_lastErrorA = error;
    m_lastTimeA = now;
  }
  else if(t_motor == B){
    unsigned long now = millis();
    double timeChange = (double)(now - m_lastTimeB);
  
    double error = m_setpointB - m_stepsB_cm; // m_stepsB_cm -> input
    m_errorSumB += (error * timeChange);
    double errorDiff = (error - m_lastErrorB) / timeChange;
    
    double proportional = m_kpB * error;
    double integral = m_kiB * m_errorSumB;
    double derivative = m_kdB * errorDiff;
  
    m_speedB = proportional + integral + derivative; // m_speedB -> output
    
    m_sendDecimalToRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC, m_speedB);
    m_sendDecimalToRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC, error);
  
    m_lastErrorB = error;
    m_lastTimeB = now;
  }
}

void Motors::m_motorSetSpeedCM(m_Motor t_motor, double t_speed){
  // Check if it is negative. If it is, change wheel direction & calculate absolute value of the speed before converting
  if(t_motor == A){
    if(t_speed < 0){
      // Move backwards
      m_motorSetDir(t_motor, CCW);
      t_speed = abs(t_speed);
    }
    else{
      // Move forwards
      m_motorSetDir(t_motor, CW);
    }
    
    int speedAPWM = int( (m_averagePWMsignalA / m_averageSpeedACMS) * t_speed);
    if(speedAPWM >= 255){
      speedAPWM = 255;
    }
    else if(speedAPWM <= 0) {
      speedAPWM = 0;
    }
    analogWrite(MOTOR_PWMA, speedAPWM);
  }
  
  else if(t_motor == B){
    if(t_speed < 0){
      // Move backwards
      m_motorSetDir(t_motor, CW);
      t_speed = abs(t_speed);
    }
    else{
      // Move forwards
      m_motorSetDir(t_motor, CCW);
    }
    
    int speedBPWM = int( (m_averagePWMsignalB / m_averageSpeedBCMS) * t_speed);
    if(speedBPWM >= 255){
      speedBPWM = 255;
    }
    else if(speedBPWM <= 0) {
      speedBPWM = 0;
    }
    analogWrite(MOTOR_PWMB, speedBPWM);
  }
}

// Helper function to set direction output and update the global direction variable
void Motors::m_motorSetDir(m_Motor t_motor, m_Direction t_dir){
  if(t_motor == A){
    digitalWrite(MOTOR_DIRA, t_dir); // Write out the direction, 0 = CW, 1 = CCW
    m_prevDirA = m_dirA;
    m_dirA = t_dir; // Update the direction variable
  }
  else if(t_motor == B){
    digitalWrite(MOTOR_DIRB, t_dir);
    m_prevDirB = m_dirB;
    m_dirB = t_dir;
  }
}

void Motors::m_setAbsoluteEncoderSteps(){
  // Absolute encoder steps
  m_sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, m_absoluteStepsA); // Send data to the register
  m_sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, m_absoluteStepsB); // Send data to the register

  int resetInputA = getRegister(REG_RECEIVE_RESET_ENCODER_A);
  int resetInputB = getRegister(REG_RECEIVE_RESET_ENCODER_B);

  if(resetInputA == 1){
    m_absoluteStepsA = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_A, 0);
  }

  if(resetInputB == 1){
    m_absoluteStepsB = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_B, 0);
  }
}

// Converts encoder steps to centimetres
double Motors::m_stepsToCentimetres(int t_steps){
  double fullRotation = 298 * 6;
  double wheelRadius = 4.75 / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

// Sends large numbers to registers (up to + or - 14bit numbers)
void Motors::m_sendEncoderStepsToRegisters(int t_register1, int t_register2, int t_steps){
  int firstPart = t_steps >> 7;
  int secondPart = t_steps - (firstPart << 7);

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

// Resets encoder values
void Motors::m_resetEncoders(){
  m_stepsA = 0;
  m_stepsB = 0;
}

// READ REGISTER FUNCTIONS

void Motors::m_readWheelDirections(){
  // Read wheel direction signals
  m_Direction registerDirA = (m_Direction)getRegister(REG_RECEIVE_DIR_MOTOR_A);
  m_Direction registerDirB = (m_Direction)getRegister(REG_RECEIVE_DIR_MOTOR_B);

  // If the direction signal for motor A has changed, change direction
  if(registerDirA != m_registerDirAprev){
    m_motorSetDir(A, registerDirA);
    m_registerDirAprev = registerDirA;
    putRegister(REG_SEND_DATA_MOTOR_A, m_dirA); // Send current wheel direction to register
  }

  // If the direction signal for motor B has changed, change direction
  if(registerDirB != m_registerDirBprev){
    m_motorSetDir(B, registerDirB);
    m_registerDirBprev = registerDirB;
    putRegister(REG_SEND_DATA_MOTOR_B, m_dirB); // Send current wheel direction to register
  }
}

void Motors::m_readPWMSignals(){
  // Read PWM signals
  int speedAPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_A);
  int speedBPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_B);

  // If the PWM signal for motor A has changed, change speed level
  if(speedAPWM != m_speedAPWM_prev){
    analogWrite(MOTOR_PWMA, speedAPWM);
    Serial.print("Adjusting motor A PWM speed to: ");
    Serial.println(speedAPWM);
    m_speedAPWM_prev = speedAPWM;
  }

  // If the PWM signal for motor B has changed, change speed level
  if(speedBPWM != m_speedBPWM_prev){
    analogWrite(MOTOR_PWMB, speedBPWM);
    Serial.print("Adjusting motor B PWM speed to: ");
    Serial.println(speedBPWM);
    m_speedBPWM_prev = speedBPWM;
  }
}

void Motors::m_readSpeedLevelValue(){
  // Get data from general robot speed register
  int speedValue = getRegister(REG_RECEIVE_SPEED_DATA);

  // If the data has changed, change speed level
  if(speedValue != m_speedValuePrev){
    m_adjustSpeed(speedValue);
    m_speedValuePrev = speedValue;
  }
}

void Motors::m_readDirectionInput(){
  // Get input data from register (WASD)
  int input = getRegister(REG_RECEIVE_MSG_DRIVE);

  // If the input has changed
  if(input != m_inputPrev){
    // Forward input
    if(input == 1){
      m_moveForward();
      putRegister(REG_SEND_MSG_DRIVE, 1);
    }
    // Backward input
    else if (input == 2){
      m_moveBackward();
      putRegister(REG_SEND_MSG_DRIVE, 2);
    }
    // Left input
    else if (input == 3){
      m_moveLeft();
      putRegister(REG_SEND_MSG_DRIVE, 3);
    }
    // Right input
    else if (input == 4){
      m_moveRight();
      putRegister(REG_SEND_MSG_DRIVE, 4);
    }
    // Anything else stops the motors
    else {
      m_stopRobot();
      putRegister(REG_SEND_MSG_DRIVE, 5);
    }
    
    m_inputPrev = input;
  }
}

// Moves the robot forwards
void Motors::m_moveForward(){
  Serial.println("Moving forwards.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CCW);
}

// Moves the robot backwards
void Motors::m_moveBackward(){
  Serial.println("Moving backwards.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CW);
}

// Moves the robot left
void Motors::m_moveLeft(){
  Serial.println("Moving left.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CCW);
}

// Moves the robot right
void Motors::m_moveRight(){
  Serial.println("Moving right.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CW);
}

// Stops the robot
void Motors::m_stopRobot(){
  Serial.println("Stopping robot.");
  m_adjustSpeed(0);
}

// Sets speed of both motors to a desired speed level (0 - 9)
void Motors::m_adjustSpeed(int t_speedLevel){
  // Variables for motor speed. Speeds are measured in PWM pulses 0 - 255
  int speedAPWM = 0; // MotorA
  int speedBPWM = 0; // MotorB
  int baseSpeed = 255 / 9; // Divides maximum pwm into 9 speed levels

  // If the speed level is between 0 and 9
  if(t_speedLevel >= 0 && t_speedLevel <= 9){
    // Set the speed to the desired level
    speedAPWM = baseSpeed * t_speedLevel;
    speedBPWM = baseSpeed * t_speedLevel;
  }
  else {
    // Stop motors
    speedAPWM = 0;
    speedBPWM = 0;
  }

  Serial.print("Adjusting motor A PWM speed to: ");
  Serial.println(speedAPWM);
  Serial.print("Adjusting motor B PWM speed to: ");
  Serial.println(speedBPWM);

  // Send speed to motors
  analogWrite(MOTOR_PWMA, speedAPWM);
  analogWrite(MOTOR_PWMB, speedBPWM);
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
    m_absoluteStepsA++;
  }
  else if(m_dirA == CCW){
    m_stepsA--;
    m_absoluteStepsA--;
  }
}

//ISR for reading MOTOR_ENCB
void Motors::m_ENCB_ISR(){
  if(m_dirB == CCW){
    m_stepsB++;
    m_absoluteStepsB++;
  }
  else if(m_dirB == CW){
    m_stepsB--;
    m_absoluteStepsB--;
  }
}
