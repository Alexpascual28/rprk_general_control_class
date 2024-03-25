#ifndef infrared_h
#define infrared_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

// I2C MUX communication
#define IR_SENSOR_ADDRESS      0x80 >> 1  // 0xEE >> 1  // 0x80 >> 1 // Address for IR sensor, shifted to provide 7-bit version
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG    0x35 // Register address to read shift value from
#define IR_BUS_NUMBER 0 // I2C bus number to read from. For the IR sensor.

// Serial registers
#define REG_SEND_IR 10 // Serial register to send the IR data to

class Infrared {
  public:
    Infrared(int t_irBusNumber);
    void initialize();
    void readIR();
    
  private:
    // Ir sensor variables
    int m_irBusNumber; // Stores ir bus number
    int m_shift; //Stores shift value from IR sensor
    int m_infraredDistancePrev = 0;

    // Member methods
    void m_initializeSerialRegisters();
    void m_setupI2C(int t_bus_number);
    int m_readI2CSensor(int t_bus_number);
};

#endif
