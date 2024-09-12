/*
	ARB.cpp - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
*/

#include <ARB.h>
#include <Wire.h>


/***
 *      __  __ _          
 *     |  \/  (_)___  ___ 
 *     | |\/| | / __|/ __|
 *     | |  | | \__ \ (__ 
 *     |_|  |_|_|___/\___|
 *                        
 */

/*
	ARBSetup - Sets up everything used by the Arduino Electronics Board
	Should be called once at the start of setup()
	This function is overloaded to provide a version that assumes serial is disabled
	or a version that takes a bool to enable or disable serial.
	
		bool serialEnable - set to true to enable serial registers
*/
void ARBSetup(void){
	Wire.begin(); // Start I2C
}
void ARBSetup(bool serialEnable){
	ARBSetup(); // Call the regular setup function first
	
	if(serialEnable){
		// Start the correct serial port if enabled, and wait for it to open
		Serial1.begin(115200);
		while(!Serial1);
	}
	

}


/*
 *      ____            _       _ 
 *     / ___|  ___ _ __(_) __ _| |
 *     \___ \ / _ \ '__| |/ _` | |
 *      ___) |  __/ |  | | (_| | |
 *     |____/ \___|_|  |_|\__,_|_|
 *                                
 */


// reg_array is an array of 128 chars (bytes) to be used for Pi<->Nano comms
char reg_array[128];

/*
	getRegister - Returns the value stored at the specified register
		int reg - the number of the requested register
		
		returns char - the requested char
*/
char getRegister(int reg){
	return reg_array[reg];
}

/*
	putRegister - Puts a given value into the specified register
		int reg - the number of the requested register
		char data - the data you want to store in that register
*/
void putRegister(int reg, char data){
	reg_array[reg] = data;
}


/*
	serialUpdate - Used to service read and write request that have come in over serialUpdate
	Should be called at least once per program loop to avoid blocking the Pi for too long
*/

void serialUpdate(){

	if(Serial1.available() > 0){ // Check if there are any bytes waiting in the buffer

		int opcode = Serial1.read(); // Read the first byte in the buffer, this will be our opcode

		if(opcode >= 128){ // If MSB of the opcode is 1, then this is a write command and we will expect a second byte
			int regNo = opcode - 128; // Extract the register number from the opcode
			while(Serial1.available() <1); // Wait for next byte
			reg_array[regNo] = Serial1.read(); // Put the next byte from the buffer into the array at the specified location
		}
		else{ // If the MSB of the opcode is 0, this is a read command
			Serial1.write(reg_array[opcode]); // Send back the specifed value from the array
		}
	}
}


/*
 *      ___ ____   ____ 
 *     |_ _|___ \ / ___|
 *      | |  __) | |    
 *      | | / __/| |___ 
 *     |___|_____|\____|
 *                      
 */

/* 
	setI2CBus - Used to configure which bus out of the I2C mux chip is currently active
	The mux chip is capable of outputing to multiple buses simultaneously
	but in this application only one bus will be active at a time.

	Once this function has been called, the user can use the standard Wire library
	commands to control the I2C devices on the bus as normal

	busNo - which bus to set active. Valid choices are 0,1,2 or 3. 
		Setting to anything else will deselect all muxed buses.
*/
void setI2CBus(int busNo){
	Wire.beginTransmission(I2C_MUX_ADDRESS); // Begin transmission to mux chip

	if(busNo >= 0 && busNo <=3){ // Only write if busNo is valid
	Wire.write(1 << busNo); // Write wanted bus to mux chip
	}
	else{ // If busNo is anything other than 0,1,2 or 3, deselect all buses
	Wire.write(0);
	}
	// The bit shift puts a 1 in the correct bit position for each bus
	// 0 = 0001, 1 = 0010, 2 = 0100, 3 = 1000

	Wire.endTransmission(); // End transmission to mux chip
  
}


/***
 *            ____              _      
 *      _   _/ ___|  ___  _ __ (_) ___ 
 *     | | | \___ \ / _ \| '_ \| |/ __|
 *     | |_| |___) | (_) | | | | | (__ 
 *      \__,_|____/ \___/|_| |_|_|\___|
 *                                     
 */

int uSecToCM(int uSec){
	// Speed of sound is approx. 340 m/s or 29 uSec per cm
	// The length of the pulse is the there and back travel time,
	// so we take half of the distance travelled
	return uSec / 29 / 2;
}
