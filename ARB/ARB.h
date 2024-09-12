/*
	ARB.h - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
*/

#ifndef ARB_H
#define ARB_H

#include <Arduino.h>

// Define easy to use names for all the pins that are used by the Arduino Robotics Board
#define SPI_SCLK D13
#define MOTOR_DIRA A0
#define MOTOR_DIRB A1
#define GPIO4 A2
#define PB_LEFT A2
#define GPIO3 A3
#define PB_DOWN A3
//#define I2C_SDA A4 // I2C already pre-defined, left here for documentation purposes
//#define I2C_SCL A5
#define GPIO2 A6
#define PB_UP A6
#define GPIO1 A7
#define PB_RIGHT A7
#define SPI_CIPO D12
#define SPI_COPI D11
#define MOTOR_PWMB D10
#define MOTOR_PWMA D9
#define SPI_CS D8
#define USONIC1 D7
#define USONIC2 D6
#define USONIC3 D5
#define USONIC4 D4
#define MOTOR_ENCB D3
#define MOTOR_ENCA D2
#define RX D1
#define TX D0

#define I2C_MUX_ADDRESS 0xEE >> 1 // Address for I2C mux, shifted to provide 7-bit version

// Variables
extern char reg_array[128];

//Function Prototypes
void ARBSetup(void);
void ARBSetup(bool serialEnable);

char getRegister(int reg);
void putRegister(int reg, char data);
void serialUpdate();

void setI2CBus(int busNo);

int uSecToCM(int uSec);

#endif