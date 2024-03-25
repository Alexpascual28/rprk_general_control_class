#ifndef ultrasonics_h
#define ultrasonics_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

#define REG_SEND_DATA_ULTRASOUND_1 20 // Ultrasound 1 send data
#define REG_SEND_DATA_ULTRASOUND_2 21 // Ultrasound 2 send data

class Ultrasonics {
  public:
    Ultrasonics();
    void initialize();
    void readUltrasonics();
    
  private:
    // Ultrasound variables
    int m_distances[2] = {0,0}; // Setup variable for results
    int m_distancesPrev[2] = {0,0};

    void m_initializeSerialRegisters();
    void m_getUltrasoundDistances();
};

#endif
