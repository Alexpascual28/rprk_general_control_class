/*
	pidControl.ino - Arduino Robotics Board
  V1 - Initial tests
	Tests for PID motor control
  Author: Alejandro Pascual San Roman (bdh532)
	Feb 2024
*/

#include "Motors.h"

#include <ARB.h>
#include <Wire.h>

Motors motors;

void setup() {
	ARBSetup(true); // Setup everything required by the board and enable serial comms
	motors.initialize();
	Serial.begin(9600); // Start serial for debugging
}

// This example loop runs each robot component sequencially
void loop() {
  motors.runMotors();
  
  // Call the serialUpdate function at least once per loop
  serialUpdate();
  delay(0.1);
}
