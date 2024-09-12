/*
	BLEPeripheral.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example shows how to use the BLE functionallity of the Arduino 33 BLE
*/

#include <ArduinoBLE.h>
#include <ARB.h>
#include <Wire.h>

// We will be reading the IR sensors as part of this example, so defining their stuff
#define IR_SENSOR_ADDRESS      0x80 >> 1 // Address for IR sensor, shifted to provide 7-bit version
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG    0x35 // Register address to read shift value from

// Timeout of 200,000uS for reading the USONIC sensors
#define USONIC_TIMEOUT 200000

int IR_prev_distance[2], IR_distance[2]; // Stores calculated distance from IR sensor
byte IR_high, IR_low; // Stores high and low byte read from IR sensor
int IR_shift[2]; // Stores shift value from IR sensor

// Stores the duration and distances from the USONIC sensors
int uS_duration[2], uS_prev_cm[2], uS_cm[2];

/* 
    These #defines define all the UUIDs in use in this sketch.
    The were generated at random using an online generator:
    https://www.uuidgenerator.net/

    For larger projects it's probably good practice to keep track of these
    in a spreadsheet somewhere so that when it comes to writing the central
    software you have an easy reference of all your UUIDs.

    It might also be an idea so seperate them out into a separate header file,
    which could be shared between the peripheral and central software.
*/
#define HELLO_WORLD_SERVICE_UUID "180A" // 0x180A = "Device Information"
#define HELLO_WORLD_TEXT_UUID "2A25" // 0x2A25 = "Serial Number String"

#define IR_RANGE_SERVICE_UUID "73150000-4179-4433-9f08-164aaa2f0de6"
#define IR_RANGE_CHARA1_UUID "73150001-4179-4433-9f08-164aaa2f0de6"
#define IR_RANGE_CHARA2_UUID "73150002-4179-4433-9f08-164aaa2f0de6"

#define USONIC_SERVICE_UUID "5a060000-0027-4090-bbcf-ca3ff3a88f63"
#define USONIC_CHARA1_UUID "5a060001-0027-4090-bbcf-ca3ff3a88f63"
#define USONIC_CHARA2_UUID "5a060002-0027-4090-bbcf-ca3ff3a88f63"

#define LED_SERVICE_UUID "7eca0000-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_R_UUID "7eca0001-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_G_UUID "7eca0002-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_B_UUID "7eca0003-67d9-4473-8015-5dbe1681ae1a"
#define LED_CHARA_MONO_UUID "7eca0004-67d9-4473-8015-5dbe1681ae1a"

// Create our services
BLEService helloWorldService(HELLO_WORLD_SERVICE_UUID);
BLEService IRRangeService(IR_RANGE_SERVICE_UUID);
BLEService uSonicService(USONIC_SERVICE_UUID);
BLEService LEDService(LED_SERVICE_UUID);

// Create our characteristics to hold our data
BLEStringCharacteristic helloWorldCharacteristic(HELLO_WORLD_TEXT_UUID, BLERead, 16);

BLEIntCharacteristic IRDistanceCharacteristic_1(IR_RANGE_CHARA1_UUID, BLERead | BLENotify);
BLEIntCharacteristic IRDistanceCharacteristic_2(IR_RANGE_CHARA2_UUID, BLERead | BLENotify);

BLEIntCharacteristic uSonicCharacteristic_1(USONIC_CHARA1_UUID, BLERead | BLENotify);
BLEIntCharacteristic uSonicCharacteristic_2(USONIC_CHARA2_UUID, BLERead | BLENotify);

BLEBoolCharacteristic LEDCharacteristic_R(LED_CHARA_R_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic LEDCharacteristic_G(LED_CHARA_G_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic LEDCharacteristic_B(LED_CHARA_B_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic LEDCharacteristic_M(LED_CHARA_MONO_UUID, BLERead | BLEWrite);

// Function prototypes
int BLESafepulseIn(int pin);

void LEDMonoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic);


void setup() {
    ARBSetup(); // Setup the ARB board
	
	// set LED pins to output mode
	pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	
	digitalWrite(LED_BUILTIN, HIGH); // This will turn the LED on
	digitalWrite(LEDR, HIGH); // will turn the LED off
	digitalWrite(LEDG, HIGH); // will turn the LED off
	digitalWrite(LEDB, HIGH); // will turn the LED off
    
    Serial.begin(9600); // Start serial for debug output

    // Initialise BLE
    if(!BLE.begin()){
        Serial.println("BLE init failed");
    }else{
        Serial.print("BLE address: ");
        Serial.println(BLE.address()); // Print the address to serial so we know where to find it
    }

    /*  Set the name of this device to be advertised
        You should set this to something unique to prevent confusion
        between different devices
     */
    BLE.setDeviceName("BLEPeripheralTest");
	BLE.setDeviceName("BLEPeripheralTest");

    // Set which service we are going to advertise
    BLE.setAdvertisedService(helloWorldService);

    // Add out characteristics to their services
	helloWorldService.addCharacteristic(helloWorldCharacteristic);
	
    IRRangeService.addCharacteristic(IRDistanceCharacteristic_1);
    IRRangeService.addCharacteristic(IRDistanceCharacteristic_2);
    
    uSonicService.addCharacteristic(uSonicCharacteristic_1);
    uSonicService.addCharacteristic(uSonicCharacteristic_2);
	
	LEDService.addCharacteristic(LEDCharacteristic_R);
	LEDService.addCharacteristic(LEDCharacteristic_G);
	LEDService.addCharacteristic(LEDCharacteristic_B);
	LEDService.addCharacteristic(LEDCharacteristic_M);

    // Add the services to the main BLE object
	BLE.addService(helloWorldService);
    BLE.addService(IRRangeService);
    BLE.addService(uSonicService);
	BLE.addService(LEDService);
	
	// Write default values to our characteristics
	helloWorldCharacteristic.writeValue("Hello World 123");
	
    IRDistanceCharacteristic_1.writeValue(0);
    IRDistanceCharacteristic_2.writeValue(0);

    uSonicCharacteristic_1.writeValue(0);
    uSonicCharacteristic_2.writeValue(0);
	
	LEDCharacteristic_R.writeValue(true);
	LEDCharacteristic_G.writeValue(true);
	LEDCharacteristic_B.writeValue(true);
	LEDCharacteristic_M.writeValue(true);
	
	/*
	    Add an event handler to the mono LED characteristic
	    This is an alternative way of handling events other than
	    just polling to see if a characteristic has been written to
    */
	
	LEDCharacteristic_M.setEventHandler(BLEWritten, LEDMonoCharacteristicWritten);
	
    // Start advertisement
    BLE.advertise();

    // Read shift value from each IR sensor in turn
    for(int i = 0; i < 2; i++){
        setI2CBus(i); // Set which bus we are reading from

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
            Serial.println(i);
        }

        // Once the data become available in the Wire bufer, put it into the shift array
        IR_shift[i] = Wire.read();

    }

}

void loop() {
    // Poll for new BLE events
    BLE.poll();

    // Request and read the two distance bytes from each IR sensor in turn
    for(int i = 0; i < 2; i++){
        // Save the previous distance so we can tell if its changed
        IR_prev_distance[i] = IR_distance[i];
        
        setI2CBus(i); // Set bus we are accessing

        // Write to sensor to tell it we are reading from the distance register
        Wire.beginTransmission(IR_SENSOR_ADDRESS);
        Wire.write(IR_SENSOR_DISTANCE_REG);
        Wire.endTransmission();

        // Request two bytes of data from the sensor
        Wire.requestFrom(IR_SENSOR_ADDRESS, 2);

        // Wait until bytes are received in the buffer
        while(Wire.available() <2);

        // Temporarily store the bytes read
        IR_high = Wire.read();
        IR_low = Wire.read();

        // Calculate the distance in cm
        IR_distance[i] = (IR_high * 16 + IR_low)/16/(int)pow(2,IR_shift[i]);
    }
    
    // Set the pin to output, bring it low, then high, then low to generate pulse
    pinMode(USONIC1, OUTPUT);
    digitalWrite(USONIC1, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC1, HIGH);
    delayMicroseconds(15);
    digitalWrite(USONIC1, LOW);

    // The same pin is used to read back the returning signal, so must be set back to input
    pinMode(USONIC1, INPUT);
    uS_duration[0] = BLESafepulseIn(USONIC1);

    // Set the pin to output, bring it low, then high, then low to generate pulse
    pinMode(USONIC2, OUTPUT);
    digitalWrite(USONIC2, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC2, HIGH);
    delayMicroseconds(15);
    digitalWrite(USONIC2, LOW);

    // The same pin is used to read back the returning signal, so must be set back to input
    pinMode(USONIC2, INPUT);
    uS_duration[1] = BLESafepulseIn(USONIC2);

    // Save old values so we can tell if they've actually changed
    uS_prev_cm[0] = uS_cm[0];
    uS_prev_cm[1] = uS_cm[1];

    // Convert to cm using helper function
    uS_cm[0] = uSecToCM(uS_duration[0]);
    uS_cm[1] = uSecToCM(uS_duration[1]);
    Serial.println(uS_cm[1]);

    /*  Update characteristics if values have changed
        Characteristics should only be updated if they have changed to avoid
        sending unnecessary notifications
    */
    if(IR_prev_distance[0] != IR_distance[0]){
        IRDistanceCharacteristic_1.writeValue(IR_distance[0]);
    }
    if(IR_prev_distance[1] != IR_distance[1]){
        IRDistanceCharacteristic_2.writeValue(IR_distance[1]);
    }

    if(uS_prev_cm[0] != uS_cm[0]){
        uSonicCharacteristic_1.writeValue(uS_cm[0]);
    }
    if(uS_prev_cm[1] != uS_cm[1]){
        uSonicCharacteristic_2.writeValue(uS_cm[1]);
    }
	
	/* 
	    Update RGB led if characteristics have been written
		This is the polling way of checking if a characteristic
		has been written to, as opposed to the handler way
	*/
	if(LEDCharacteristic_R.written()){
		digitalWrite(LEDR, LEDCharacteristic_R.value());
	}
	if(LEDCharacteristic_G.written()){
		digitalWrite(LEDG, LEDCharacteristic_G.value());
	}
	if(LEDCharacteristic_B.written()){
		digitalWrite(LEDB, LEDCharacteristic_B.value());
	}

}


/*
    Helper function to replace the pulseIn function as it's not compatible
	with the ArduinoBLE library. This way is less accurate than the pulseIn
	function, but works without breaking BLE.
*/
int BLESafepulseIn(int pin){
    int previousMicros = micros();
    while(!digitalRead(pin) && (micros() - previousMicros) <= USONIC_TIMEOUT); // wait for the echo pin HIGH or timeout
    previousMicros = micros();
    while(digitalRead(pin)  && (micros() - previousMicros) <= USONIC_TIMEOUT); // wait for the echo pin LOW or timeout
    return micros() - previousMicros; // duration
}

void LEDMonoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic){
	if(LEDCharacteristic_M.value()){
		digitalWrite(LED_BUILTIN, HIGH);
	} else{
		digitalWrite(LED_BUILTIN, LOW);
	}
}
