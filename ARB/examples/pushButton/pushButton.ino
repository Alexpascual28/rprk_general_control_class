/*
	pushButton.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example shows one way of reading button inputs from the onboard 5-way button
*/

#include <ARB.h>

// Define struct to hold button push flags
volatile struct ButtonState{
	bool left = false;
	bool right = false;
	bool up = false;
	bool down = false;
} buttons;

void setup() {
	
	ARBSetup(); // Setup ARB functionallity

	// Set pins to inputs
	pinMode(PB_LEFT, INPUT);
	pinMode(PB_RIGHT, INPUT);
	pinMode(PB_UP, INPUT);
	pinMode(PB_DOWN, INPUT);

	// Enable internal pull-ups
	digitalWrite(PB_LEFT, HIGH);
	digitalWrite(PB_RIGHT, HIGH);
	digitalWrite(PB_UP, HIGH);
	digitalWrite(PB_DOWN, HIGH);

	/* 
	  Attach interrupts to PB pins
	  The internal pull-ups pul the pins high when the button is open.
	  When the button is closed it grounds the pin, so we want to be 
	  looking for falling edges to see when the button has been pressed.
	*/
	attachInterrupt(digitalPinToInterrupt(PB_LEFT), LEFT_ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(PB_RIGHT), RIGHT_ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(PB_UP), UP_ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(PB_DOWN), DOWN_ISR, FALLING);


	// Start serial for debugging
	Serial.begin(9600);
}

/*
  This basic loop checks if a button press flag has been raised
  and if it has it prints out the name of that button and clears
  the flag
*/
void loop() {
  if(buttons.left == true){
    Serial.println("left");
    buttons.left = false;
  }
  
  if(buttons.right == true){
    Serial.println("right");
    buttons.right = false;
  }
  
  if(buttons.up == true){
    Serial.println("up");
    buttons.up = false;
  }
  
  if(buttons.down == true){
    Serial.println("down");
    buttons.down = false;
  }

}
/*
  It's best practice to keep ISRs as short as possible so these ISRs
  just set a flag if a button has been pressed so it can be acted upon
  outside of the ISR
*/
void LEFT_ISR(){
  buttons.left = true;
}

void RIGHT_ISR(){
  buttons.right = true;
}

void UP_ISR(){
  buttons.up = true;
}

void DOWN_ISR(){
  buttons.down = true;
}