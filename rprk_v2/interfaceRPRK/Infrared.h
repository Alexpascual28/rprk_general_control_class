#ifndef infrared_h
#define infrared_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

// I2C MUX communication
#define IR_SENSOR_ADDRESS      0x80 >> 1  // 0xEE >> 1  // 0x80 >> 1 // Address for IR sensor, shifted to provide 7-bit version
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG    0x35 // Register address to read shift value from
#define IR_1_BUS_NUMBER 0 // I2C bus number to read from. For the IR sensor.
#define IR_2_BUS_NUMBER 1 // I2C bus number to read from. For the IR sensor.
#define IR_3_BUS_NUMBER 2 // I2C bus number to read from. For the IR sensor.
#define IR_4_BUS_NUMBER 3 // I2C bus number to read from. For the IR sensor.

// Serial registers
#define REG_SEND_IR_1 10 // Serial register to send the IR data to IR 1
#define REG_SEND_IR_2 11 // Serial register to send the IR data to IR 2
#define REG_SEND_IR_3 12 // Serial register to send the IR data to IR 3
#define REG_SEND_IR_4 13 // Serial register to send the IR data to IR 4

class Infrared {
  public:
    Infrared(int t_irBusNumber);
    void initialize();
    int readIR();
    
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
