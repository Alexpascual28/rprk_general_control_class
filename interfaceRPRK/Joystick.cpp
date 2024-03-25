#include "Joystick.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

Joystick::Joystick(){
}

// STATIC MEMBER VARIABLE INITIALISATION

volatile Joystick::m_ButtonState Joystick::m_buttons;

// PUBLIC METHODS

void Joystick::initialize(){
  m_setPinModes();
  m_initializeComponents(); // Initializes values
  m_attachInterrupts(); // Attaches interrupts
  m_initializeSerialRegisters(); // Initialize serial registers
}

void Joystick::readInput(){
  // Read Joystick (orientations are inverted, if the point of reference is forward=up)
  if(m_buttons.left == true){
    Serial.println("Joystick right");
    putRegister(REG_SEND_DATA_JOYSTICK, 4);
    m_buttons.left = false;
  }
  
  if(m_buttons.right == true){
    Serial.println("Joystick left");
    putRegister(REG_SEND_DATA_JOYSTICK, 3);
    m_buttons.right = false;
  }
  
  if(m_buttons.up == true){
    Serial.println("Joystick down");
    putRegister(REG_SEND_DATA_JOYSTICK, 2);
    m_buttons.up = false;
  }
  
  if(m_buttons.down == true){
    Serial.println("Joystick up");
    putRegister(REG_SEND_DATA_JOYSTICK, 1);
    m_buttons.down = false;
  }
}

// PRIVATE MEMBERS

void Joystick::m_setPinModes(){
  // Set pins to inputs
  pinMode(PB_LEFT, INPUT);
  pinMode(PB_RIGHT, INPUT);
  pinMode(PB_UP, INPUT);
  pinMode(PB_DOWN, INPUT);
}

void Joystick::m_initializeComponents(){
  // Enable internal pull-ups
  digitalWrite(PB_LEFT, HIGH);
  digitalWrite(PB_RIGHT, HIGH);
  digitalWrite(PB_UP, HIGH);
  digitalWrite(PB_DOWN, HIGH);
}

void Joystick::m_attachInterrupts(){
  /* 
    Attach interrupts to PB pins
    The internal pull-ups pul the pins high when the button is open.
    When the button is closed it grounds the pin, so we want to be 
    looking for falling edges to see when the button has been pressed.
  */
  attachInterrupt(digitalPinToInterrupt(PB_LEFT), m_LEFT_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_RIGHT), m_RIGHT_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_UP), m_UP_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_DOWN), m_DOWN_ISR, FALLING);
}

void Joystick::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi

  // Joystick
  putRegister(REG_SEND_MSG_JOYSTICK, 0); // Joystick send messages
  putRegister(REG_RECEIVE_MSG_JOYSTICK, 0); // Joystic receive messages
  putRegister(REG_SEND_DATA_JOYSTICK, 0); // Joystick send data (direction)
}

/*
  It's best practice to keep ISRs as short as possible so these ISRs
  just set a flag if a button has been pressed so it can be acted upon
  outside of the ISR
*/
void Joystick::m_LEFT_ISR(){
  m_buttons.left = true;
}

void Joystick::m_RIGHT_ISR(){
  m_buttons.right = true;
}

void Joystick::m_UP_ISR(){
  m_buttons.up = true;
}

void Joystick::m_DOWN_ISR(){
  m_buttons.down = true;
}
