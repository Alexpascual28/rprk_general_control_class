#include "Infrared.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

Infrared::Infrared(int t_irBusNumber){
  m_irBusNumber = t_irBusNumber;
}

// PUBLIC METHODS

void Infrared::initialize(){
  m_initializeSerialRegisters(); // Initialize serial registers
  m_setupI2C(m_irBusNumber);
}

int Infrared::readIR(){
  // Read the IR sensor
  int infraredDistance = m_readI2CSensor(m_irBusNumber);
  int serialRegister = (m_irBusNumber == IR_1_BUS_NUMBER) ? REG_SEND_IR_1 : ((m_irBusNumber == IR_2_BUS_NUMBER) ? REG_SEND_IR_2 : ((m_irBusNumber == IR_3_BUS_NUMBER) ? REG_SEND_IR_3 : REG_SEND_IR_4));

  if (infraredDistance != m_infraredDistancePrev){
    putRegister(serialRegister, infraredDistance);
    m_infraredDistancePrev = infraredDistance;
  }
  
  return infraredDistance;
}

// PRIVATE MEMBERS

void Infrared::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi
  // IR sensor
  putRegister(REG_SEND_IR_1, 0);
  putRegister(REG_SEND_IR_2, 0);
  putRegister(REG_SEND_IR_3, 0);
  putRegister(REG_SEND_IR_4, 0);

}

// I2C
void Infrared::m_setupI2C(int t_bus_number) {
  setI2CBus(t_bus_number); // Set which bus we are reading from

  // Write to the sensor to tell it we are reading from the shift register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_SHIFT_REG);
  Wire.endTransmission();

  // Request 1 byte of data from the sensor to read the shift register
  Wire.requestFrom(IR_SENSOR_ADDRESS, 1);

  while(Wire.available() == 0){
    // Tells the user if the sketch is waiting for a particular sensor
    // If sensor does not reply, it may be a sign of a faulty or disconnected sensor
    Serial.print("Waiting for sensor ");
    Serial.print(t_bus_number);
    Serial.println();
  }

  // Once the data become available in the Wire bufer, put it into the shift array
  m_shift = Wire.read();
}

int Infrared::m_readI2CSensor(int t_bus_number){
  setI2CBus(t_bus_number); // Set bus we are accessing

  // Write to sensor to tell it we are reading from the distance register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_DISTANCE_REG);
  Wire.endTransmission();

  // Request two bytes of data from the sensor
  Wire.requestFrom(IR_SENSOR_ADDRESS, 2);

  // Wait until bytes are received in the buffer
  while(Wire.available() <2);

  // Temporarily store the bytes read. Stores high and low byte read
  byte high = Wire.read();
  byte low = Wire.read();

  // Calculate the distance in cm
  int distance = (high * 16 + low)/16/(int)pow(2,m_shift); // Stores calculated distance

  // Print out values over serial
  // Serial.print("IR sensor distance is ");
  // Serial.print(distance);
  // Serial.print("cm.\n");

  return distance;
}
