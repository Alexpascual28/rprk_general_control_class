/*
	uSonic.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example reads the distance from two HC-SR04 Ultrasonic range sensors
	and outputs their measured distance in cm over the debug serial
	
	The sensors should be connected to USONIC1 and USONIC2
*/

#include <ARB.h>

void setup(){
	ARBSetup(); // Setup ARB functionallity
	Serial.begin(9600); // Startup serial for debugging
}

void loop(){
	int duration[2], cm[2]; // Setup variables for results

	// Set the pin to output, bring it low, then high, then low to generate pulse
	pinMode(USONIC1, OUTPUT);
	digitalWrite(USONIC1, LOW);
	delayMicroseconds(2);
	digitalWrite(USONIC1, HIGH);
	delayMicroseconds(15);
	digitalWrite(USONIC1, LOW);

	// The same pin is used to read back the returning signal, so must be set back to input
	pinMode(USONIC1, INPUT);
	duration[0] = pulseIn(USONIC1, HIGH);

	// Set the pin to output, bring it low, then high, then low to generate pulse
	pinMode(USONIC2, OUTPUT);
	digitalWrite(USONIC2, LOW);
	delayMicroseconds(2);
	digitalWrite(USONIC2, HIGH);
	delayMicroseconds(15);
	digitalWrite(USONIC2, LOW);

	// The same pin is used to read back the returning signal, so must be set back to input
	pinMode(USONIC2, INPUT);
	duration[1] = pulseIn(USONIC2, HIGH);

	// Convert to cm using helper function
	cm[0] = uSecToCM(duration[0]);
	cm[1] = uSecToCM(duration[1]);

	Serial.print("Distance 1: ");
	Serial.print(cm[0]);
	Serial.println(" cm.");

	Serial.print("Distance 2: ");
	Serial.print(cm[1]);
	Serial.println(" cm.");
	
	delay(50); // Short delay before next loop
}