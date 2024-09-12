/*
	motorControl.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example shows how the two motor channels can be controlled,
	and the steps from the encoders can be recorded
*/

#include <ARB.h>

// Define some enums to make specifying motor and direction easier
typedef enum {CW,CCW} Direction; // CW = 0, CCW = 1
typedef enum {A,B} Motor;

// Global variables for motor direction so they can be read by encoder ISR to determine direction
Direction dirA = CW;
Direction dirB = CW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int stepsA = 0;
volatile int stepsB = 0;

void setup() {
	
	ARBSetup(); // Setup ARB functionallity

	// Set relevant modes for pins
	pinMode(MOTOR_DIRA, OUTPUT);
	pinMode(MOTOR_DIRB, OUTPUT);
	pinMode(MOTOR_PWMA, OUTPUT);
	pinMode(MOTOR_PWMB, OUTPUT);
	pinMode(MOTOR_ENCA, INPUT);
	pinMode(MOTOR_ENCB, INPUT);

	// Set motors off by default
	motorSetDir(A, CW);
	motorSetDir(B, CW);
	analogWrite(MOTOR_PWMA, 0);
	analogWrite(MOTOR_PWMB, 0);

	// Attach interrupts to motor encoder inputs so we don't miss any steps
	attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
	attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING

	Serial.begin(9600); // Start serial for debugging
  
}


// This example loop turns each motor on at 50% for 2 seconds in each direction
// It is recommended that the robot is propped up so the wheels do not touch the ground for this test
void loop() {

	// Print the current number of steps recorded
	Serial.print("A steps: ");
	Serial.println(stepsA);
	Serial.print("B steps: ");
	Serial.println(stepsB);

	Serial.println("All Off");

	// Set direction to CW and speed to 0
	motorSetDir(A, CW);
	motorSetDir(B, CW);
	analogWrite(MOTOR_PWMA, 0);
	analogWrite(MOTOR_PWMB, 0);

	delay(2000); // Wait 2 secs

	Serial.println("Motor A on 50%");

	analogWrite(MOTOR_PWMA, 127); // analogWrite is 0-255 so 127 = 50% speed

	delay(2000);

	Serial.print("A steps: ");
	Serial.println(stepsA);
	Serial.print("B steps: ");
	Serial.println(stepsB);

	Serial.println("Motor A change dir");

	motorSetDir(A, CCW); // Change direction of motor A to CCW

	delay(2000);

	Serial.print("A steps: ");
	Serial.println(stepsA);
	Serial.print("B steps: ");
	Serial.println(stepsB);

	Serial.println("Motor A off, Motor B 50%");

	analogWrite(MOTOR_PWMA, 0); // Zero speed turns off motor
	analogWrite(MOTOR_PWMB, 128);

	delay(2000);

	Serial.print("A steps: ");
	Serial.println(stepsA);
	Serial.print("B steps: ");
	Serial.println(stepsB);

	Serial.println("Motor B change dir");

	motorSetDir(B, CCW);

	delay(2000);
  
}

// Helper function to set direction output and update the global direction variable
void motorSetDir(Motor motor, Direction dir){
	if(motor == A){
		digitalWrite(MOTOR_DIRA, dir); // Write out the direction, 0 = CW, 1 = CCW
		dirA = dir; // Update the direction variable
	}
	else if(motor == B){
		digitalWrite(MOTOR_DIRB, dir);
		dirB = dir;
	}
}


/* 
  Since we know the direction the motors should be travelling in, we do not need
  the full quadrature output of the encoders, we only take one channel from each
  to save on pins.
  
  These ISRs will be called once per edge, either rising or falling, and will add
  to the number of steps if the motor is currently moving CW, and take away if CCW
*/

//ISR for reading MOTOR_ENCA
void ENCA_ISR(){
	if(dirA == CW){
		stepsA++;
	}
	else if(dirA == CCW){
		stepsA--;
	}
  
}

//ISR for reading MOTOR_ENCB
void ENCB_ISR(){
	if(dirB == CW){
		stepsB++;
	}
	else if(dirB == CCW){
		stepsB--;
	}
}
