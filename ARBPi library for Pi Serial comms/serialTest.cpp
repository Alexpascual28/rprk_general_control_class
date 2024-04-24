/*
	serialTest.cpp - Arduino Robotics Board
	A Python library to compliment the ARB Arduino library
	when using the ARB platfrom in conjuncion with a Raspberry Pi
	June 2022
	
	This example should run in conjunction with the serialComms
	example on the Arduino side of things.
	It opens a serial channel to the Arduino, read several registers
	and then writes to a register.
*/

#include <iostream>
#include "ARBPi.h"

using namespace std;

int main(){
	ARBPiSetup();
	
	// Read the first 5 registers and print their content
	cout << "Read test. Reading reg 0-4" << endl;	
	cout << getRegister(0) << endl;
	cout << getRegister(1) << endl;
	cout << getRegister(2) << endl;
	cout << getRegister(3) << endl;
	cout << getRegister(4) << endl;

	// Read the 30th register, write to it, then read it back
	cout << "Write test. Reading reg 30" << endl;
	cout << getRegister(30) << endl;
	cout << "Writing '@' to reg 30" << endl;
	putRegister(30, '@');
	cout << "Reading reg 30 again" << endl;
	cout << getRegister(30) << endl;
}
