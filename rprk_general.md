<p align="center">
  <img src="https://raw.githubusercontent.com/PKief/vscode-material-icon-theme/ec559a9f6bfd399b82bb44393651661b08aaf7ba/icons/folder-markdown-open.svg" width="100" alt="project-logo">
</p>
<p align="center">
    <h1 align="center">RPRK General Control Class</h1>
</p>
<p align="center">
    <em><code>RPRK (Raspberry Pi Robotics Kit) – Serial communication interface algorithm between Raspberry Pi 4 and Arduino Nano 33 BLE through UART for a custom "RPRK" mobile robot that solves a maze. Generalised modular C++ control class to interface with platform sensors and send data through serial registers, couples with a general Python class to receive data and send commands.</code></em>
</p>

<br><!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary><br>

- [ Overview](#-overview)
- [ Features](#-features)
- [ Repository Structure](#-repository-structure)
- [ Modules](#-modules)
- [ Getting Started](#-getting-started)
  - [ Installation](#-installation)
  - [ Usage](#-usage)
  - [ Tests](#-tests)
- [ Project Roadmap](#-project-roadmap)
- [ Contributing](#-contributing)
- [ License](#-license)
- [ Acknowledgments](#-acknowledgments)
</details>
<hr>

##  Overview

<code>The RPRK (Raspberry Pi Robotics Kit) is a  to build a mobile robot equipped with various sensors and actuators managed by a Raspberry Pi Model 4 and an Arduino Nano 33 BLE via the custom Arduino Robotics Board (ARB). These sessions guide students through the fundamentals of robotics, including sensor integration, motor control, and serial communications, culminating in the development of a robot that can navigate and map an environment autonomously.</code>

<code>This project is designed for students and developers interested in the fields of robotics and software engineering, using Raspberry Pi, Arduino, Python, and C++. It contains a set of practical lab sessions that allow hands-on experience with real-world robotic systems programming and control.</code>

<code>The focus is on interfacing and controlling various robotic functionalities such as motor control, sensor integration, and visual processing using OpenCV. The project is structured in a way to guide the user from simple to more complex robotics applications, making it suitable for educational purposes or for hobbyists looking to enhance their robotics skills.</code>

---

##  Features

<code>► INSERT-TEXT-HERE</code>

---

##  Repository Structure

```sh
└── rprk_general_control_class/
    ├── README.md
    ├── RPRK.py
    ├── getch_test.py
    ├── interfaceRPRK
    │   ├── Infrared.cpp
    │   ├── Infrared.h
    │   ├── Joystick.cpp
    │   ├── Joystick.h
    │   ├── Motors.cpp
    │   ├── Motors.h
    │   ├── Ultrasonics.cpp
    │   ├── Ultrasonics.h
    │   └── interfaceRPRK.ino
    ├── keyboard_control_with_avoidance.py
    ├── left_wall_follower.py
    ├── pidControl
    │   ├── Motors.cpp
    │   ├── Motors.h
    │   └── pidControl.ino
    └── pid_wheel_control.py
```

---

##  Modules

<details closed><summary>.</summary>

| File                                                                                                                                                 | Summary                         |
| ---                                                                                                                                                  | ---                             |
| [RPRK.py](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/RPRK.py)                                                       | <code>► INSERT-TEXT-HERE</code> |
| [keyboard_control_with_avoidance.py](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/keyboard_control_with_avoidance.py) | <code>► INSERT-TEXT-HERE</code> |
| [left_wall_follower.py](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/left_wall_follower.py)                           | <code>► INSERT-TEXT-HERE</code> |
| [pid_wheel_control.py](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/pid_wheel_control.py)                             | <code>► INSERT-TEXT-HERE</code> |
| [getch_test.py](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/getch_test.py)                                           | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>interfaceRPRK</summary>

| File                                                                                                                             | Summary                         |
| ---                                                                                                                              | ---                             |
| [Infrared.cpp](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Infrared.cpp)           | <code>► INSERT-TEXT-HERE</code> |
| [Infrared.h](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Infrared.h)               | <code>► INSERT-TEXT-HERE</code> |
| [Motors.cpp](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Motors.cpp)               | <code>► INSERT-TEXT-HERE</code> |
| [Ultrasonics.cpp](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Ultrasonics.cpp)     | <code>► INSERT-TEXT-HERE</code> |
| [Joystick.cpp](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Joystick.cpp)           | <code>► INSERT-TEXT-HERE</code> |
| [interfaceRPRK.ino](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/interfaceRPRK.ino) | <code>► INSERT-TEXT-HERE</code> |
| [Joystick.h](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Joystick.h)               | <code>► INSERT-TEXT-HERE</code> |
| [Motors.h](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Motors.h)                   | <code>► INSERT-TEXT-HERE</code> |
| [Ultrasonics.h](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/interfaceRPRK/Ultrasonics.h)         | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>pidControl</summary>

| File                                                                                                                    | Summary                         |
| ---                                                                                                                     | ---                             |
| [Motors.cpp](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/pidControl/Motors.cpp)         | <code>► INSERT-TEXT-HERE</code> |
| [Motors.h](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/pidControl/Motors.h)             | <code>► INSERT-TEXT-HERE</code> |
| [pidControl.ino](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/master/pidControl/pidControl.ino) | <code>► INSERT-TEXT-HERE</code> |

</details>

---

##  Getting Started

**Software and Libraries**

To run this project, you will need:

For the Arduino:

* Arduino IDE
* C++ Standard Library
* ARB library (included, explained below)

On the Raspberry Pi:

* Python 3.6 or above
* Python Packages: `numpy`, `picamera`, `picamera.array`, `cv2`, `aruco`, `matplotlib`, `ctypes`, `csv`, `os`, `threading`

On your computer:

* PuTTY: To establish comms with the Pi using *SSH*. (or equivalent)
* WinSCP: For FTP comms and file transfer. (or equivalent)
* XMing: To forward camera image data. (or equivalent)

**Hardware**

The Raspberry Pi Robotics Kit (RPRK) is designed to help students build and program a mobile robot capable of solving a maze through localization and mapping. The robot uses various sensors, including wheel encoders, a front-facing camera, two side ultrasonic sensors, and a front infrared sensor, to navigate its environment. The kit includes:

* **Arduino Robotics Board (ARB)**: A custom board based on the *Arduino Nano 33 BLE*, featuring motor drivers, connectors for IR distance sensors, and headers for ultrasonic sensors.
* **Arduino Nano 33 BLE**: Integrated into the **ARB**.
* **Raspberry Pi Model 4**: Available with varying RAM from 2GB to 8GB, with 2GB being common and sufficient for the project.
* **USB to UART HAT**: Installed on the Raspberry Pi for serial console access.
* **MicroSD card**: Preloaded with a custom *Raspbian Buster* image containing necessary software.
* **Raspberry Pi Camera 3**: Comes with a ribbon cable.
* **USB battery bank** and **Micro USB and USB-C cables**.
* **Motors and Wheels**: Includes two 298:1 micro metal gearmotors with encoders, mounted in 3D-printed red mounts, and two narrow wheels.
* **Pololu 3/4" ball caster**: Includes mounting hardware, noting that the screws are imperial and not interchangeable with metric screws.
* **Sensor and Motor Driver kit**: Contains an IR sensor, a motor driver, and two ultrasonic range sensors.
* **Assorted 3D printed and laser cut parts**: Includes mounts for sensors and chassis components.
* **Assorted Velcro strips and fasteners**: For mounting and assembly.

###  Installation

1. **Clone the repository** to your local machine or **download the entire project directory**.

   <h4>From <code>source</code></h4>

   > 1. Clone the rprk_general_control_class repository:
   >
   > ```console
   > $ git clone https://github.com/Alexpascual28/rprk_general_control_class.git
   > ```
   >
   > 2. Change to the project directory:
   > ```console
   > $ cd rprk_general_control_class
   > ```

2. **Install the ARB Library** on your Arduino:
   1. In the *Arduino IDE*, go to the menu bar and select **Sketch** > **Include Library** > **Add .ZIP Library....**
   2. Zip the ARB directory and navigate to where you have saved your **"ARB.zip"** file.
   3. Select the file and click on **'Open'**. The IDE will then install the library.
   4. Verify Installation:
      - To check if the library has been successfully installed, go back to **Sketch** > **Include Library**. You should see the library named "ARB" at the bottom of the drop-down menu.
      - Click on it to include the library in your current sketch, which should automatically insert an include statement like `#include <ARB.h>` at the top of your sketch.

3. **Open a connection to the Raspberry Pi** using PuTTY or directly through HDMI (Windows instructions). [Lab Sessions: Lab 1 PDF](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf):
   1. Connect to the Raspberry Pi with your laptop using the serial **USB to UART HAT** and a USB cable.
   2. Check what *COM* port the device is connected to using "Device Manager"
   3. Establish a `Serial` connection with PuTTY using the device *COM* port and baud rate 115200.
   
   Alternatively, you can connect a screen and keyboard using the micro HDMI and USB-A ports in the Raspberry Pi directly to the Raspberry Pi to access the terminal directly.

   4. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **Username:** *pi*
      * **Password:** *raspberry*

4. **Connect the Pi to a local WiFi** network and **check the device's IP address** on the network.
   1. Type `sudo raspi-config` in the command line to open the configuration screen.
   2. Go to **“2: Network Options”** and then **“N2 Wireless LAN”** and enter the SSID and passphrase for your network. You must connect the Pi to the same network as your computer. For the lab network, use the following details:
      * **SSID:** *robotlab*
      * **Password:** *vetzlentath*
   3. Go to "Finish" and wait a few moments for the Raspberry Pi to connect.
   4. Type `ifconfig` on the terminal.
   5. Look for the section called *wlan0*. You should see your IP address there (e.g 144.32.70.210).
   6. Take note of your IP, it can be used to connect to the board through SSH or to transfer files with FTP. You can now close the serial PuTTY or direct connection.

5. **Connect through SSH using PuTTY** (Windows instructions)
   1. Open PuTTY again. You must be connected to the same network as the Raspberry Pi.
   2. Establish an `SSH` connection using host name *"username@ip_address"* (e.g pi@144.32.70.210) and port 22, using the previously established IP address.
   3. To forward camera image data from the Pi to your computer, you must:
      * Have [XMing](http://www.straightrunning.com/XmingNotes/) installed in your device. Further instructions in [Lab Sessions: Lab 6 PDF](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab6/autumn_lab_6.pdf)
      * Execute XMing before establishing a connection. It will run in the background.
      * In PuTTY, go to **Connections** > **SSH** > **X11** and check the box that says *'Enable X-11 forwarding'*.
   4. If you wish, save the session under your preferred name by going to **Session** > **"Load, save or delete a stored session"**. Write your session name under *Saved Sessions* and click **"Save"**.
   5. Click **"Open"** at the bottom-right of the window.
   6. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **Username:** *pi*
      * **Password:** *raspberry*

6. **View, add and modify files using WinSCP** (Windows instructions). Further instructions in [Lab Sessions: Lab 1 PDF](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/Lab1/autumn_lab_1.pdf)
   1. Open WinSCP. You must be connected to the same network as the Raspberry Pi.
   2. Create a "New Site" with the following details:
      * **File Protocol**: *SFTP*
      * **Host Name**: The device's IP address for the network in format *XXX.XX.XX.XXX* (e.g *144.32.70.210*). Refer to step 4 in [Installation](#installation).
      * **Port**: *22*
      * **User Name**: Your username for the Raspberry Pi. In lab devices: ***pi***.
      * **Password**: Your password for the Raspberry Pi. In lab devices: ***raspberry***.
   3. Click "Login". After a connection is established you should be able to see the files in the Pi.

7. **Upload all project files** to the Raspberry Pi using WinSCP.

8. **Install Python dependencies** with the CLI in SSH PuTTY:

   All of the Python libraries can be installed using pip:

   ```bash
   pip install numpy opencv-python picamera matplotlib ...
   ```

###  Usage

**Running the code**

To run Arduino files, upload the respective `.ino` sketch files to the Arduino Nano using the Arduino IDE.

For the Raspberry Pi, navigate to the specific target file and run the Python scripts through the terminal:

```bash
python3 <script_name>.py
```

Ensure that the `ARBPi` file and the `RPRK` files or other dependencies are located in the same folder when running a script that requires their functions.

Ensure that the Raspberry Pi and ARB are connected via UART, as the scripts and sketches often communicate over this channel.

**To run the left wall follower**

1. Upload the `interfaceRPRK.ino` sketch into the Arduino.
2. Open **XMing** and make sure **X11 forwarding** is enabled as per the instructions above.
3. In your Raspberry Pi, run:

   ```bash
   python3 left_wall_follower.py
   ```

4. Place the RPRK robot in the obstacle course. It will attempt to complete the maze by following the wall on its left. The robot's camera feed with annotations should pop-up in your screen.


**To run the WASD keyboard controller**

1. Upload the `interfaceRPRK.ino` sketch into the Arduino.
2. Open **XMing** and make sure **X11 forwarding** is enabled as per the instructions above.
3. In your Raspberry Pi, run:

   ```bash
   python3 keyboard_control_with_avoidance.py
   ```

4. Place the RPRK robot in a flat surface or the obstacle course. The robot's camera feed with annotations should pop-up in your screen. You should be able to control the robot by using the WASD keys on your keyboard and the digits 0-9 in your numberpad to choose the robot's speed by level.

**To run the PID controller**

1. Upload the `interfaceRPRK.ino` or the `pidControl.ino` sketch into the Arduino.
2. In your Raspberry Pi, run:

   ```bash
   python3 pid_wheel_control.py
   ```

4. Place the RPRK robot in in a flat surface or the obstacle course. The robot should move forward 10 centimetres as per the code.

---

##  ARB and ARBPi

The ARB (Arduino Robotics Board) and ARBPi libraries are crucial for communication between the Arduino and Raspberry Pi. They handle low-level operations like reading and writing to registers that control motors, read sensors, and manage other peripherals.

### ARB library

The ARB (Arduino Robotics Board) library runs in the Arduino Nano and is designed to simplify the interaction between the hardware components on the ARB and the software controlling it, in conjunction with a Raspberry Pi. This library is integral for controlling the motors, sensors, and serial communication in your robotics projects.

**Key Components of the ARB Library**

*Header File (`ARB.h`)*:

* Defines all the necessary pins used by the ARB with easy-to-understand names such as **MOTOR_DIRA** for motor direction control and **USONIC1** to **USONIC4** for ultrasonic sensors.
* It includes standard Arduino headers and defines constants for the **I2C multiplexer address** and various **GPIO** pins.
* Declares an external array `reg_array` of 128 bytes to manage data communication between the Raspberry Pi and the Arduino.
* Provides function prototypes for initialization (`ARBSetup`), register manipulation (`getRegister`, `putRegister`), and I2C bus management (`setI2CBus`).

*Source File (`ARB.cpp`)*:

* Implements the functions declared in the header. The `ARBSetup` function initializes **I2C** communication and optionally the serial communication depending on the passed parameter.
* The `getRegister` and `putRegister` functions manage data in the `reg_array`, facilitating communication between the Arduino and any connected device like the Raspberry Pi.
* The `setI2CBus` function controls which bus on the **I2C multiplexer** is active, allowing the selection of different sensor sets connected to the Arduino.
* Provides a utility function `uSecToCM` to convert time (in microseconds) to distance (in centimeters), which is useful for ultrasonic distance measurements.

**`getRegister` and `putRegister` Functions in the ARB Library**

The `getRegister` and `putRegister` functions are crucial components of the ARB library, allowing for efficient data communication between the Arduino and other devices, such as the Raspberry Pi. These functions manage data within an array named `reg_array`, which acts as a collection of registers used to store and retrieve data dynamically during runtime.

*`getRegister` Function*

The `getRegister` function is designed to access data from the `reg_array`. Here's how it works:

* **Prototype**: char getRegister(int reg);
* **Parameters**: It takes a single integer reg, which represents the index of the register in the * reg_array from which data is to be retrieved.
* **Returns**: The function returns a char value, which is the data stored at the specified register index.

   **Code Snippet**:

   ```cpp
   // C++
   char getRegister(int reg){
      return reg_array[reg];
   }
   ```

   **Usage**:

   This function is typically used when you need to read a value from a specific register that may have been written to by another part of your program or from an external device like the Raspberry Pi. For instance, you might store sensor data or configuration settings in these registers.

*`putRegister` Function*

The `putRegister` function allows writing data to a specific register in the `reg_array`. Here’s a detailed look:

* **Prototype**: void putRegister(int reg, char data);
* **Parameters**:
   * `int reg`: The index of the register where the data will be stored.
   * `char data`: The data to be stored in the register.

   **Code Snippet**:

   ```cpp
   // C++
   void putRegister(int reg, char data){
      reg_array[reg] = data;
   }
   ```

   **Usage**:

   This function is crucial for updating the contents of a register, which could influence the behavior of the robot or other parts of the system. For example, it could be used to update control parameters, set desired motor speeds, or store temporary data needed for computations.

**Example Usage**

Here’s a basic example of how the ARB library functions might be used in an Arduino sketch:

```cpp
// C++
#include <ARB.h>

void setup() {
    ARBSetup(true); // Initialize with serial communication enabled
}

void loop() {
    // Example of setting a register value
    putRegister(0, 120); // Put 120 in register 0

    // Example of reading a register value
    char val = getRegister(0);

    // Use the infrared sensor connected to I2C bus 1
    setI2CBus(1);
}
```

Here is how you might use both getRegister and putRegister in a practical scenario:

```cpp
// C++
void setup() {
    ARBSetup(); // Initialize ARB without serial communication
}

void loop() {
    // Setting a register value to store a motor speed setting
    putRegister(10, 50);  // Assume register 10 is designated for motor speed

    // Later in the loop, or in another function, you retrieve this motor speed
    char motorSpeed = getRegister(10);

    // Use the motor speed to control a motor
    analogWrite(MOTOR_PWMA, motorSpeed);  // Assuming MOTOR_PWMA controls a motor's speed
}
```

A demo usage of the ARB library for serial communication can also be found in `examples/serialComms.ino`.

**Overview of Example Files in the ARB Examples Directory**

Each example in the ARB library's **"examples"** directory demonstrates specific functionalities of the Arduino Robotics Board. Here is a brief overview of what each example illustrates:

1. BLEPeripheral (`BLEPeripheral.ino`)
This example showcases how to set up and use a Bluetooth Low Energy (BLE) peripheral with the Arduino. It is essential for projects that require wireless data transmission or remote control via BLE.

2. I2CMux (`I2CMux.ino`)
Demonstrates the use of an I2C multiplexer with the ARB. This is critical for projects where multiple I2C devices must share the same I2C bus without addressing conflicts.

3. motorControl (`motorControl.ino`)
Provides a basic example of how to control motors using the ARB. It includes setting up the motor drivers and controlling the speed and direction of DC motors, which is fundamental for any mobile robotics project.

4. pushButton (`pushButton.ino`)
Shows how to read the state of push buttons using the ARB. It is useful for projects that require user input or a simple interface for triggering actions.

5. serialComms (`serialComms.ino`)
This example is about setting up and using serial communication between the ARB and another device, like a Raspberry Pi or a computer. It covers sending and receiving data over serial, which is vital for debugging and complex communications.

6. uSonic (`uSonic.ino`)
Focuses on using ultrasonic sensors with the ARB to measure distances. This is particularly useful in robotics for obstacle avoidance, navigation, and environment mapping.

**Detailed Explanation of Key Examples**

Let's dive deeper into two specific examples: `motorControl` and `uSonic`.

*`motorControl.ino`*

This script initializes and controls two DC motors connected to the ARB. It handles setting the direction and speed of each motor through PWM signals, which are essential for driving the motors in forward or reverse directions.

Key Snippets:

```cpp
// C++
void setup() {
    pinMode(MOTOR_PWMA, OUTPUT);  // Set motor A PWM pin as output
    pinMode(MOTOR_DIRA, OUTPUT);  // Set motor A direction pin as output
}

void loop() {
    analogWrite(MOTOR_PWMA, 128);  // Set speed for motor A
    digitalWrite(MOTOR_DIRA, HIGH); // Set direction for motor A
    delay(1000);                    // Run for 1 second
    digitalWrite(MOTOR_DIRA, LOW);  // Change direction
    delay(1000);                    // Run in the opposite direction for 1 second
}
```

*`uSonic.ino`*

This script demonstrates how to use an ultrasonic sensor connected to the ARB to measure distances. The script calculates the distance by timing how long it takes for an ultrasonic pulse to return to the sensor.

Key Snippets:

```cpp
// C++
void setup() {
    pinMode(USONIC1, INPUT);  // Set the ultrasonic sensor pin as input
}

void loop() {
    long duration, distance;
    digitalWrite(USONIC1, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC1, HIGH);
    delayMicroseconds(10);
    digitalWrite(USONIC1, LOW);
    duration = pulseIn(USONIC1, HIGH);
    distance = duration / 29 / 2;  // Calculate distance
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(1000);
}
```

### ARBPi library

**Purpose and Functionality:**

The ARBPi library runs in the Raspberry Pi and provides serial communication between the Raspberry Pi and the Arduino boards. The library offers a dual-interface, supporting both C++ and Python, thereby accommodating a wide range of programming preferences and project requirements.

**Technical Stack and Integration:**

**C++ Components**: Core functionalities are implemented in C++, ensuring high performance and direct access to low-level system resources.

**Python Interface**: Python bindings are provided to leverage the ease of scripting and rapid development capabilities of Python, making it ideal for higher-level applications and quick testing.

**Key Components:**

*C++ Source Files (`ARBPi.cpp` and `ARBPi.h`)*:
   - `ARBPi.cpp`: Contains the implementation of serial communication functions such as setting up the serial port, reading and writing to registers on the **Arduino**.
   - `ARBPi.h`: Header file that declares the functions and constants used by `ARBPi.cpp`.

*Python Module (`ARBPi.py`)*:
   Wraps the **C++** library using **Python’s** ctypes module, providing Pythonic access to the underlying serial communication functions.

*Compiled Library (`libARBPi.so`)*:
   A shared library compiled from the **C++** code, enabling dynamic linking from Python or other **C++** programs.

*Test Files (`serialtest`, `serialTest.cpp`, and `serialTest.py`)*:
   Executable and scripts for testing the functionality of the library to ensure proper operation of serial communications.

*Libraries and Dependencies*:
   - `wiringPi`: Used in the **C++** code for handling **GPIO** and serial communications on the Raspberry Pi.
   - `ctypes`: Utilized in the **Python** script to interface with the **C++** shared library.

**Setup and Usage Instructions:**

*Compiling C++ Code:*

To compile the C++ part of the library, you would use g++ with appropriate flags to link against necessary libraries, such as wiringPi:

```bash
g++ -o ARBPi ARBPi.cpp -lwiringPi
```

*Running Python Scripts:*

Ensure Python is installed along with ctypes. The Python script can be run by importing it to your code as follows:

```python
# Python
from ARBPi import *
```

**Initialization Process:**

C++ and Python interfaces include an initialization function to set up the serial connection:
```cpp
// C++
void ARBPiSetup() {
    serialDevice = serialOpen(SERIAL, 115200);
}
```
```python
# Python
def ARBPiSetup(serialPath="/dev/ttyUSB0"):
    _ARBPi.ARBPiSetup(ctypes.c_char_p(serialPath.encode('ascii')))
```

Reading and Writing Registers:

To read a register:
```cpp
// C++
char getRegister(int reg) {
    serialPutchar(serialDevice, reg);
    while(serialDataAvail(serialDevice) < 1) {}
    return serialGetchar(serialDevice);
}
```
```python
# Python
def getRegister(reg):
    return int(_ARBPi.getRegister(ctypes.c_int(reg)))
```

To write to a register:
```cpp
// C++
void putRegister(int reg, char data) {
    serialPutchar(serialDevice, reg + 128);
    serialPutchar(serialDevice, data);
}
```
```python
# Python
def putRegister(reg, data):
    _ARBPi.putRegister(ctypes.c_int(reg), ctypes.c_byte(data))
```

These snippets illustrate the direct interaction with hardware through serial interfaces, encapsulating complex operations into simple, reusable API calls.

---

##  Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Report Issues](https://github.com/Alexpascual28/rprk_general_control_class.git/issues)**: Submit bugs found or log feature requests for the `rprk_general_control_class` project.
- **[Submit Pull Requests](https://github.com/Alexpascual28/rprk_general_control_class.git/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.
- **[Join the Discussions](https://github.com/Alexpascual28/rprk_general_control_class.git/discussions)**: Share your insights, provide feedback, or ask questions.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your github account.
2. **Clone Locally**: Clone the forked repository to your local machine using a git client.
   ```sh
   git clone https://github.com/Alexpascual28/rprk_general_control_class.git
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to github**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.
8. **Review**: Once your PR is reviewed and approved, it will be merged into the main branch. Congratulations on your contribution!
</details>

<details closed>
<summary>Contributor Graph</summary>
<br>
<p align="center">
   <a href="https://github.com{/Alexpascual28/rprk_general_control_class.git/}graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Alexpascual28/rprk_general_control_class.git">
   </a>
</p>
</details>

---

##  License

This project is protected under the [SELECT-A-LICENSE](https://choosealicense.com/licenses) License. For more details, refer to the [LICENSE](https://choosealicense.com/licenses/) file.

---

##  Acknowledgments

- List any resources, contributors, inspiration, etc. here.

[**Return**](#-overview)

---
