/*
	ARBPi.cpp - Arduino Robotics Board
	A Python library to compliment the ARB Arduino library
	when using the ARB platfrom in conjuncion with a Raspberry Pi
	June 2022
*/

#include "ARBPi.h"

/***
 *      __  __ _          
 *     |  \/  (_)___  ___ 
 *     | |\/| | / __|/ __|
 *     | |  | | \__ \ (__ 
 *     |_|  |_|_|___/\___|
 *                        
 */

// Define an int to hold the Linux file descriptor of our selected serial device
int serialDevice;

/*
	ARBPiSetup - Sets up everything required by the library
	Should be called once at the start of the program
	
		char serialPath[] - the path to the serial device to use
			SERIAL is a macro for the correct path for USB serial
*/
void ARBPiSetup(){
	serialDevice = serialOpen(SERIAL, 115200);
}

/*
 *      ____            _       _ 
 *     / ___|  ___ _ __(_) __ _| |
 *     \___ \ / _ \ '__| |/ _` | |
 *      ___) |  __/ |  | | (_| | |
 *     |____/ \___|_|  |_|\__,_|_|
 *                                
 */

/*
	getRegister - Gets the value stored in a register on the Nano
	This is a blocking function because it will wait for a reply from the 
	Nano before continuing
	
		int reg - the number of the register to read
	
		
		returns char - the byte stored in the register
*/
char getRegister(int reg){
	serialPutchar(serialDevice, reg); // Send the register number over the serial

	while(serialDataAvail(serialDevice) < 1){} // Wait until there is a byte waiting in the receive buffer

	return serialGetchar(serialDevice); // Get the byte from the buffer and return it
}


/*
	putRegister - Puts a value into a register on the Nano
	This is not a blocking function as it sends the data out but does not
	wait for any reply back
	
		int reg - the number of the register to write to
		char data - the byte of data to write to the selected register
*/
void putRegister(int reg, char data){
        serialPutchar(serialDevice, reg + 128); // Send the number of the register + 128 to signify a write
        serialPutchar(serialDevice, data); // Send the data to be written
}

