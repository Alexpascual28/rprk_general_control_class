#include "Ultrasonics.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

Ultrasonics::Ultrasonics(){
}

// PUBLIC METHODS

void Ultrasonics::initialize(){
  m_initializeSerialRegisters(); // Initialize serial registers
}

void Ultrasonics::readUltrasonics(){
  // Read ultrasonic sensors
  m_getUltrasoundDistances();

  if(m_distances[0] != m_distancesPrev[0]){
    putRegister(REG_SEND_DATA_ULTRASOUND_1, m_distances[0]);
    m_distancesPrev[0] = m_distances[0];
  }

  if(m_distances[1] != m_distancesPrev[1]){
    putRegister(REG_SEND_DATA_ULTRASOUND_2, m_distances[1]);
    m_distancesPrev[1] = m_distances[1];
  }
}

// PRIVATE MEMBERS

void Ultrasonics::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi
  // Ultrasound
  putRegister(REG_SEND_DATA_ULTRASOUND_1, 0);
  putRegister(REG_SEND_DATA_ULTRASOUND_2, 0);
}

void Ultrasonics::m_getUltrasoundDistances(){
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

  m_distances[0] = cm[0];
  m_distances[1] = cm[1];
}
