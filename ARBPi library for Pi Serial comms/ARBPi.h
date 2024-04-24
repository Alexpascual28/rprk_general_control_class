/*
	ARBPi.h - Arduino Robotics Board
	A Python library to compliment the ARB Arduino library
	when using the ARB platfrom in conjuncion with a Raspberry Pi
	June 2022
*/

#ifndef ARBPI_H
#define ARBPI_H

// Include the wiringPi libraries we are using for serial
#include <wiringPi.h>
#include <wiringSerial.h>

//define the path to access the serial device
//this should be consistant as long as the Nano is the only USB serial device
//this also leaves the GPIO UART free for other use if required
#define SERIAL "/dev/ttyUSB0"

//declaring functions
extern int serialDevice;

// Function prototypes
// These need to be extern'd so they are accessible from the Python library

extern "C"{
	void ARBPiSetup();
	char getRegister(int reg);
	void putRegister(int reg, char data);
}

#endif
