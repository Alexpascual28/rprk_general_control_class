#ifndef joystick_h
#define joystick_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

#define REG_SEND_MSG_JOYSTICK 60 // Joystick send messages
#define REG_RECEIVE_MSG_JOYSTICK 61 // Joystic receive messages
#define REG_SEND_DATA_JOYSTICK 62 // Joystick send data (direction)

class Joystick {
  public:
    Joystick();
    void initialize();
    void readInput();
    
  private:
    // Define struct to hold button push flags
    struct m_ButtonState{
      bool left = false;
      bool right = false;
      bool up = false;
      bool down = false;
    };
    
    volatile static m_ButtonState m_buttons;

    void m_setPinModes();
    void m_initializeComponents(); // Initializes values
    void m_attachInterrupts(); // Attaches interrupts
    void m_initializeSerialRegisters(); // Initialize serial

    static void m_LEFT_ISR();
    static void m_RIGHT_ISR();
    static void m_UP_ISR();
    static void m_DOWN_ISR();
};

#endif
