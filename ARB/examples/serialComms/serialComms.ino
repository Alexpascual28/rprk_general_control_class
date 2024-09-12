/*
	serialComms.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example demonstrates the use of the serial communication between the Arduino
	and the Raspberry Pi and is intended to run with the companion Raspberry Pi example.
*/

#include <ARB.h>
  
void setup(){
	ARBSetup(true); // Setup everything required by the board and enable serial comms
	
	// Setup some dummy data in the registers to be read by the Raspberry Pi
	putRegister(0, 'h');
	putRegister(1, 'e');
	putRegister(2, 'l');
	putRegister(3, 'l');
	putRegister(4, 'o');
  	putRegister(5, 999);
}

void loop(){
	// Main user code loop would go here
	
	// Call the serialUpdate function at least once per loop
	serialUpdate();
}
