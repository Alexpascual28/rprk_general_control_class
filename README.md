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

<code>The RPRK (Raspberry Pi Robotics Kit) is a kit for robotics development. Using the parts in the kit, the user can build a mobile robot and program an algorithm that uses localisation and mapping to solve a maze. It includes wheel encoders, a camera and some sensors (2 ultrasonics at both sides, one infrared in front and one camera in front) to navigate the environment.</code>

<code>This repo offers a serial communication algorithm to control the RPRK platform. This system utilizes two main modules, the **RPRK** Python library (**RPRK.py**) and the **RPRK Interface** Arduino sketch (<b>interfaceRPRK.ino</b>).</code>

<code>The **RPRK.py** class and the **interfaceRPRK.ino** sketch collaboratively manage the **RPRK** robotics platform, utilizing the Raspberry Pi for high-level decision-making and the Arduino Nano 33 BLE for real-time hardware interaction. The **interfaceRPRK.ino** on the Arduino processes and relays sensor data and actuator commands via serial registers, serving as the primary interface for hardware control. Concurrently, the **RPRK Python class** on the Raspberry Pi abstracts the robot's operations, sending commands through the **ARBPi** library, which handles serial communications. This setup ensures seamless integration and communication between the Raspberry Pi and Arduino, leveraging respective libraries (**ARB** for Arduino and **ARBPi** for Raspberry Pi) to facilitate effective control and coordination of the robotic system.</code>

---

##  Directory Structure

The project includes several key directories and files:

* **ARB.zip**: Contains the Arduino library for managing serial communications on the Arduino.
* **ARBPi library for Pi Serial comms/**: Directory with the Python library for handling serial communications on the Raspberry Pi.
* **interfaceRPRK/**: Folder containing the Arduino sketch `interfaceRPRK.ino` which serves as the control interface on the Arduino.
* **pidControl/**: Folder containing scripts and resources related to PID control logic.
* `ARBPi.py`: A Python script that includes functionality for interfacing with the Arduino through serial communication.
* `libARBPi.so`: Shared library used by the ARBPi Python scripts for serial communication.
* `getch_test.py`: Test script for verifying keyboard inputs.
* `keyboard_control_with_avoidance.py`: Python script to control the robot using keyboard inputs while avoiding obstacles.
* `left_wall_follower.py`: Python algorithm for maze solving by following the left wall.
* `pid_wheel_control.py`: Implements PID control for wheel motion.
* `RPRK.py`: Python class providing high-level abstraction for robot control and interfacing.
* `README.md`: General information and documentation about the RPRK project.

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

* **Arduino Robotics Board (ARB)**: A custom board based on the *Arduino Nano 33 BLE*, featuring motor drivers, a joystick, connectors for IR distance sensors, and headers for ultrasonic sensors.
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

The ARB (Arduino Robotics Board) library runs in the Arduino Nano and is designed to simplify the interaction between the hardware components on the ARB and the software controlling it, in conjunction with a Raspberry Pi. This library is integral for controlling the motors, sensors, and serial communication in your robotics projects. For a more detailed explanation, check [RPRK Lab Sessions: ARB Library](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/README.md#arb-library)

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

### ARBPi library

**Purpose and Functionality:**

The ARBPi library runs in the Raspberry Pi and provides serial communication between the Raspberry Pi and the Arduino boards. The library offers a dual-interface, supporting both C++ and Python, thereby accommodating a wide range of programming preferences and project requirements. For a more detailed explanation, check [RPRK Lab Sessions: ARBPi Library](https://github.com/Alexpascual28/rprk_turtlebot_lab_sessions/blob/main/README.md#arbpi-library)

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

**Using in Python Scripts:**

Ensure Python is installed along with ctypes. The Python script can be run by importing it to your code as follows:

```python
# Python
from ARBPi import *
```

*Process:*

C++ and Python interfaces include an initialization function to set up the serial connection:
```python
# Python
def ARBPiSetup(serialPath="/dev/ttyUSB0"):
    _ARBPi.ARBPiSetup(ctypes.c_char_p(serialPath.encode('ascii')))
```

**Reading and Writing Registers:**

To read a register:
```python
# Python
def getRegister(reg):
    return int(_ARBPi.getRegister(ctypes.c_int(reg)))
```

To write to a register:
```python
# Python
def putRegister(reg, data):
    _ARBPi.putRegister(ctypes.c_int(reg), ctypes.c_byte(data))
```

---

# "RPRK" and "interfaceRPRK" Classes

The `RPRK` class and the `interfaceRPRK.ino` sketch form a system for controlling the RPRK robotics platform, using the Raspberry Pi and Arduino Nano 33 BLE, respectively. High-level decision-making and processing are handled by the Raspberry Pi, while real-time hardware interactions are managed by the Arduino, combining the strengths of both platforms for effective robot control.

The `interfaceRPRK.ino` reads all sensor data and forwards it through specified serial registers, while at the same time reading the data recieved through separate control registers and operating the actuators based on it. It thus, acts as a general control interface for the Arduino. The `RPRK` python class uses serial communication to create a high-level abstraction of the RPRK robot's operation. It can be imported into any Python project in order to send commands to the ARB board by the use of simple functions. These functions send the appropiate signal through the right serial channel in order for `interfaceRPRK.ino` to receive this signal and operate the robot's actuators and sensors in accordance with the command.

To establish and maintain serial communication between the ARB and the Pi, two libraries are provided. The `ARB.zip` library is designed to be imported into Arduino, and it is used by `interfaceRPRK.ino` to manage serial comms on the Arduino side. The `ARBPi` library is designed to be imported into the Python or C script in the Pi to handle comms on the Raspberry side.

## RPRK Class (Raspberry Pi)

The RPRK class (`RPRK.py`) is a Python module designed for handling various components of the Raspberry Pi Robotics Kit (RPRK). This module interfaces with Arduino via the use of the `ARBPi` serial communication library for sensor and actuator management, processing sensor data, and controlling actuators based on sensor inputs and predefined algorithms.

The `RPRK.py` file encapsulates multiple classes and functionalities to enable comprehensive robot control and monitoring. This script contains the main class `RPRK`, which integrates various sub-modules like *motors*, *camera*, *infrared* and *ultrasonic* sensors, and *joystick* control. Each submodule is encapsulated in its nested class within the `RPRK` class, allowing for organized and modular programming. Here's a detailed breakdown of its components and functionalities:

### Initialisation

To initialize the RPRK system, create an instance of the RPRK class. This setup will connect to the Arduino through serial communication, initialize all connected sensors, and prepare the motors and camera for operation.

```python
# Python
from RPRK import RPRK

robot = RPRK()
```

* `__init__`: The constructor initializes the communication setup and declares the initial position of the robot. It initializes serial communication with Arduino using the `ARBPiSetup()` function from the ARBPi module. It also instantiates the sub-classes that manage different parts of the robot such as motors, camera, infrared sensor, ultrasonic sensors, and joystick.

```python
#Python
def __init__(self):
        # Setup the ARB functions
        print("Setting up ARB")
        ARBPiSetup(SERIAL)
        
        self.initial_pose = [0, 0, 0] #  x, y, w(orientation)
        
        self.motors = self.Motors(self, self.initial_pose)
        self.camera = self.Camera()
        self.infrared = self.InfraredSensor()
        self.ultrasound = self.Ultrasonic()
        self.joystick = self.Joystick()
```

### Helper Functions

The RPRK class includes several helper functions designed to facilitate common tasks related to reading from and writing to hardware registers. These functions are crucial for handling the data between the Raspberry Pi and the Arduino in a structured and efficient manner. 

1. `read_16bit_number(register1, register2)`

   This function is used to read two separate 8-bit values from the specified registers and combine them into a single 16-bit signed integer. The process involves:

   * Fetching the first part from register2 and the second part from register1.
   * Shifting the first part by 7 bits to the left and adding the second part to construct the full 16-bit value.

   Here is how it's implemented:

   ```python
   # Python
   def read_16bit_number(self, register1, register2):
      first_part = getRegister(register2)
      second_part = getRegister(register1)
      
      result = (first_part << 7) + second_part
      return result
   ```

   This function is essential for operations where data exceeds the capacity of a single 8-bit register, such as reading encoder values which can span a wider range than 8 bits can offer.

2. `send_16bit_number(number, register1, register2)`

   The counterpart to `read_16bit_number`, this function splits a 16-bit number into two 8-bit parts and sends each part to a specified register. It operates by:

   * Shifting the number right by 7 bits to isolate the upper part.
   * Subtracting the shifted value (left shifted back by 7 bits) from the original number to get the lower part.
   * Writing each part to its corresponding register.

   Here’s the implementation:

   ```python
   # Python
   def send_16bit_number(self, number, register1, register2):
      first_part = number >> 7
      second_part = number - (first_part << 7)
      
      putRegister(register1, second_part)
      putRegister(register2, first_part)
   ```

   This function is crucial for sending data to the Arduino that requires more than 8 bits, such as sending precise control signals to actuators.

3. `read_fractional_number(register1, register2)`

   This function reads two values from specified registers, interpreting them as the whole part and fractional part of a decimal number, respectively. It:

   * Retrieves the whole part from register1.
   * Retrieves the fractional part from register2, which represents the decimal places.
   * Combines these to form a decimal number.

   Here is how it works:

   ```python
   # Python
   def read_fractional_number(self, register1, register2):
      wholePart = getRegister(register1)
      fractPart = getRegister(register2)
      
      resultFrac = wholePart + fractPart/100
      return resultFrac
   ```

   This function is particularly useful for precise measurements that require decimal accuracy but are transmitted by the Arduino in separate whole and fractional parts, such as sensor readings.

4. `send_fractional_number(number, register1, register2)`

   The inverse of `read_fractional_number`, this function takes a decimal number and splits it into a whole part and a fractional part, sending each to a designated register. It:

   * Converts the number into an integer to isolate the whole part.
   * Multiplies the decimal part by 100 to convert it into a 2-digit integer suitable for transmission.
   * Sends each part to the corresponding register.

   Example usage is as follows:

   ```python
   # Python
   def send_fractional_number(self, number, register1, register2):
      wholePart = int(number)
      fractPart = int((number - wholePart) * 100)
      
      putRegister(register1, wholePart)
      putRegister(register2, fractPart)
   ```

   This function is used to send detailed decimal values, such as PID setpoints or calibration data, where precision is critical.

### Motors (`Motors`) Submodule

**Attributes**: The `Motors` class nested within `RPRK` contains attributes for wheel distances, speed, and various register addresses for interfacing with the Arduino for motor control.

**Methods**: Includes methods to change direction, PWM speed control, set speed levels, adjust individual wheel speeds, PID control, read and reset encoder values, and compute current robot pose based on wheel encoder readings, utilizing encoder feedback to adjust the movement dynamically.

**Initialisation**

1. `__init__`: The `__init__` function within the `Motors` subclass of the RPRK class serves as the constructor for initializing motor-related attributes and settings. This function is called automatically when a new instance of the Motors class is created.

* It is responsible for initializing the motor system of the robot. It sets up important attributes such as the distances between wheels, movement steps, and speed settings for individual motors. The function also establishes serial register mappings used for sending commands and reading data from the hardware, as well as setting the initial pose of the robot.

**Direct Movement Control**

These functions interface directly with the robot's hardware. They send commands to specific registers that control the motors' speed and direction. Each function employs the `getRegister` and `putRegister` methods from the **ARB** library to read from or write to the Arduino’s registers.

2. `change_direction(self, direction)`

   This function allows the robot to change its overall movement direction. It takes a single argument direction, which specifies the intended movement direction of the robot. The possible directions are *'forward'*, *'backward'*, *'left'*, *'right'*, and *'stop'*. Here’s how it works:

   * **Mapping Directions**: A dictionary maps these string directions to numerical values understood by the Arduino hardware. This values will be read by the "`Motors.cpp`" class within `interfaceRPRK.ino` (explained later).
   * **Command Execution**: It sends the corresponding numerical command to a specific register that controls the robot's direction.
   * **Feedback**: Prints the direction the robot is moving towards or indicates if the input was incorrect.

   ```python
   # Changes the robot direction based on global direction commands
   def change_direction(self, direction):
      possible_directions = {"forward": 1, "backward": 2, "left": 3, "right": 4, "stop": 5}
      
      if direction in possible_directions:
            # Send data
            putRegister(self.REG_RECEIVE_MSG_DRIVE, possible_directions.get(direction))
            print(f"Moving {direction}")
      else:
            print("Incorrect direction.")
   ```

3. `set_robot_speed_by_level(self, speed_level)`

   This function adjusts the robot's speed based on predefined speed levels ranging from 0 to 9, where each level represents a specific speed setting:

   * **Validation**: Checks if the speed_level input is within the acceptable range (0-9).
   * **Command Execution**: Sends the speed level as a command to a register that adjusts the robot's speed.
   * **Feedback**: Provides confirmation of the new speed setting or error if the input level is out of range.

4. `set_wheel_speed(self, wheel, speed_cm_s)`

   This function sets the speed of a specified wheel (either **'A'** or **'B'**) to a specific value in centimeters per second (cm/s):

   * **Calculate PWM**: Converts the desired speed in cm/s to a PWM signal value based on pre-determined calibration factors.
   * **Validation**: Ensures the PWM value stays within the maximum allowable limits (0-255).
   * **Command Execution**: Sends the calculated PWM value to the corresponding motor's register.
   * **Feedback**: Prints the set speed and corresponding PWM value or indicates incorrect wheel input.

5. `set_wheel_direction(self, wheel, direction)`

   This function controls the rotational direction of a specified wheel:

   * **Direction Definitions**: Maps *'CW'* (clockwise) and *'CCW'* (counterclockwise) to respective hardware-understood numerical values.
   * **Command Execution**: Sends the direction command to the appropriate register controlling the specified wheel's direction.
   * **Feedback**: Confirms the direction set for the wheel or prints an error for incorrect inputs.

6. `set_robot_speed(self, speed_a, speed_b=None)`

   This function simultaneously sets the speeds of both wheels:

   * **Dual Speed Setting**: If `speed_b` is not specified, it sets both wheels to the speed of `speed_a`. If `speed_b` is given, each wheel gets its respective speed setting.
   * **Command Execution**: Utilizes `set_wheel_speed` internally to send commands to both wheels based on the speeds provided.
   * **Flexibility**: Allows independent speed control for each wheel, which is useful for maneuvers like turning.

   These functions are integral to the control system of the robot, providing direct manipulation of its movement, speed, and direction, enabling precise navigation and task execution within its operational environment.

   ```python
   # Python
   # Sets speed of both wheels simultaneously
   def set_robot_speed(self, speed_a, speed_b=None):
      if speed_b == None:
            self.set_wheel_speed("A", speed_a);
            self.set_wheel_speed("B", speed_a);
      else:
            self.set_wheel_speed("A", speed_a);
            self.set_wheel_speed("B", speed_b);
   ```

**Encoder Control**

These methods are designed for specific tasks involving motor encoder control and the robot's movement. They interact with hardware at a low level, retrieving and manipulating data critical for maintaining the robot's spatial awareness and operational accuracy.

7. `get_current_steps(self, encoder)`

   This function reads the current step count from a specified wheel encoder. Encoders on motors A and B measure the number of steps the wheels have taken, which is essential for tracking distance traveled and for feedback control systems.

   * **Parameter**: `encoder` specifies which encoder (*A* or *B*) to read from.
   * **Process**: It retrieves the step count by reading two separate registers (**high** and **low** byte) for the specified encoder and combining them into a single value using the `read_16bit_number` method.
   * **Output**: Returns the current step count as an integer.

   ```python
   # Reads current steps from the encoder registers
   def get_current_steps(self, encoder):
      possible_encoders = {"A": [self.REG_SEND_DATA_ENCODER_A_1, self.REG_SEND_DATA_ENCODER_A_2],
                           "B": [self.REG_SEND_DATA_ENCODER_B_1, self.REG_SEND_DATA_ENCODER_B_2]}
      
      if encoder in possible_encoders:
            r1, r2 = possible_encoders.get(encoder)
            steps = self.rprk.read_16bit_number(r1, r2)
            return steps
      else:
            print("Incorrect encoder.")
   ```

8. `get_current_distance(self, encoder)`

   This function calculates and retrieves the current distance traveled by the robot, based on the readings from the specified encoder (*A* or *B*).

   * **Parameter**: `encoder` identifies which wheel's encoder to use for the distance calculation.
   * **Process**: Reads the whole and fractional parts of the distance from the designated registers and combines them into a single decimal value using the `read_fractional_number` method.
   * **Output**: Returns the current distance traveled in units consistent with those programmed into the encoders (usually *centimeters*).

9. `get_current_speed(self, encoder)`

   This function calculates and returns the current speed of the robot based on encoder data. It uses the readings from the encoder specified (*A* or *B*) to determine how fast the wheel is moving.

   * **Parameter**: `encoder` specifies which wheel's encoder to use.
   * **Process**: Similar to `get_current_distance`, it retrieves the speed as a fractional number from two registers representing the whole and fractional parts of the speed.
   * **Output**: Returns the speed of the wheel in units per second, providing real-time speed data which is crucial for dynamic speed adjustments.

   ```python
   # Reads current robot speed from the encoder registers
   def get_current_speed(self, encoder):
      possible_encoders = {"A": [self.REG_SEND_SPEED_A, self.REG_SEND_SPEED_A_DEC],
                           "B": [self.REG_SEND_SPEED_B, self.REG_SEND_SPEED_B_DEC]}
      
      if encoder in possible_encoders:
            r1, r2 = possible_encoders.get(encoder)
            speed = self.rprk.read_fractional_number(r1, r2)
            return speed
      else:
            print("Incorrect encoder.")
   ```

10. `reset_encoder(self, encoder)`

      This function resets the step count of a specified encoder to zero. Resetting encoders is necessary when initializing the robot or when you need to recalibrate or restart distance measurements.

      * **Parameter**: `encoder` indicates which encoder (*A* or *B*) to reset.
      * **Process**: Sends a reset command to the appropriate *register* linked to the chosen encoder.
      * **Output**: Typically does not return a value but resets the encoder's step count, allowing for fresh measurements from a known reference point.

**Odometry**

These functions are responsible for calculating the robot's current position and speed based on encoder data. They are esential for dynamic movement control and navigation.

11. `calculate_current_pose(self, distance_a, distance_b)`

      This function updates the robot's current pose (*position* and *orientation*) based on the distances traveled by each wheel, as measured by the encoders. The pose includes the robot's coordinates (*x*, *y*) and its orientation (angle *theta*).

      * **Parameters**: `distance_a`: Distance traveled by wheel A. `distance_b`: Distance traveled by wheel B.
      * **Process**:
         * **Calculating Angular Change**: It first calculates the change in angle (`angle_change`) as (`distance_b` - `distance_a`) / `self.distance_between_wheels`, which gives the difference in wheel distances divided by the axle length.
         * **Calculating Linear Movement**: The function then computes the average linear movement (`distance_change`) as (`distance_a` + `distance_b`) / 2.
         * **Updating Pose**: The new position (*x*, *y*) and orientation (*theta*) are updated based on the previous pose and the calculated angular and linear changes. This update uses trigonometric calculations to determine how much the robot has moved and rotated.
         * **Output**: Returns the incremental changes (`distance_change`, `angle_change`) and updates the robot's internal pose representation.

12. `calculate_speed_cm_s(self, distance_cm, time_difference_s)`

      This function calculates the speed of the robot in centimeters per second based on the distance traveled over a given time interval. It's useful for monitoring and controlling the robot's speed dynamically.

      * **Parameters**: `distance_cm`: The distance traveled in centimeters. `time_difference_s`: The time elapsed during the travel in seconds.
      * **Process**: Speed Calculation. Computes the speed as `distance_cm` / `time_difference_s`, which represents the average speed over the specified time period.
      * **Output**: Returns the calculated speed in centimeters per second.

**PID Control**

These functions are used to interface with the Arduino's PID control functions and parameters (in `Motors.cpp`). They are responsible for setting up and adjusting feedback control that adapts the robot's behavior based on ongoing performance metrics.

13. `set_control_mode(self, control_mode)`

      This function sets the control mode of the motors. It is used to switch between different modes of motor operation such as **PID** control or direct speed control, based on the tasks or conditions the robot is handling.

      * **Parameter**: `control_mode`, A string that specifies the desired control mode (`"PID"` for *PID* control or `"SPEED"` for *direct speed* control).
      * **Process**:
         * A dictionary maps the string identifiers to numerical values understood by the motor controller hardware.
         * The function sends the appropriate numerical value to a control register that dictates the control mode of the motors.

      ```python
      # Method to set control mode
      def set_control_mode(self, control_mode):
         control_modes = {"PID": 1, "SPEED": 0}

         if control_mode in control_modes:
               # Send data
               putRegister(self.REG_RECEIVE_CONTROL_MODE, control_modes.get(control_mode))
               print(f"Control mode: {control_mode}")
         else:
               print("Incorrect mode.")
      ```

14. `set_pid_tuning(self, motor, tuning, value)`

      This function adjusts the **PID** (*Proportional-Integral-Derivative*) coefficients for a specified motor. It's critical for optimizing control responses such as stability, responsiveness, and steady-state error in robotic movements.

      * **Parameters**:
         * `motor`: Specifies which motor (*'A'* or *'B'*) the tuning applies to.
         * `tuning`: Specifies which *PID* parameter to set (`"KP"` for the proportional gain, `"KI"` for the integral gain, or `"KD"` for the derivative gain).
         * `value`: The numerical value to set for the specified PID parameter.

      * **Process**: Maps the tuning parameter to the appropriate control register associated with the motor. Sends the value to the specified register.

15. `set_pid_tunings(self, motor, kp, ki, kd)`

      This function sets all the **PID** tuning parameters simultaneously for a specific motor. It's used for more comprehensive adjustments to the **PID** controller settings, allowing simultaneous updates to all parameters for consistent control behavior.

      * **Parameters**:
         * `motor`: Specifies which motor (*'A'* or *'B'*) the tunings apply to.
         * `kp`: The value for the proportional gain.
         * `ki`: The value for the integral gain.
         * `kd`: The value for the derivative gain.
      * **Process**: Utilizes the `set_pid_tuning` method to set each of the PID parameters individually but within a single operation.

      ```python
      # Method to set PID tunings
      def set_pid_tunings(self, motor, kp, ki, kd):
         self.set_pid_tuning(motor, "KP", kp)
         self.set_pid_tuning(motor, "KI", ki)
         self.set_pid_tuning(motor, "KD", kd)
      ```

16. `set_pid_setpoint(self, motor, value)`

      This function sets the PID **setpoint** for a particular motor. The *setpoint* is the target state for the PID controller, such as a desired speed or position, against which the actual output is compared. In this case, the setpoint represents the target position of the motor.

      * **Parameters**:
         * `motor`: Identifies which motor (*'A'* or *'B'*) the setpoint is for.
         * `value`: The target setpoint value.
      * **Process**: The value is sent to two registers: one for the whole part and one for the fractional part of the setpoint, to maintain precision. This split allows for fine-grained control over the setpoint, accommodating both integer and decimal values.

Camera Class:
Initialization: Configures the Raspberry Pi camera and prepares it for continuous image capture.
Image Processing: Methods for detecting markers (Aruco), blobs, and colors within the camera feed. Uses OpenCV for image analysis.

Ultrasonic Class:
Sensor Reading: Provides a method to retrieve the distance measurement from ultrasonic sensors.

Joystick Class:
Joystick Control: Methods to get the current direction from joystick inputs.

InfraredSensor Class:
Distance Measurement: Method to fetch the current distance from an infrared sensor.

Detailed Methodologies
read_16bit_number and send_16bit_number:
These methods facilitate reading and writing of 16-bit integers across two 8-bit registers, critical for handling larger data values that exceed the capacity of a single 8-bit register.
read_fractional_number and send_fractional_number:
These methods manage floating-point numbers by splitting them into integer and fractional parts, crucial for precise control and measurement tasks.
Use Cases in Robotics
Motor Control: Adjust speeds and directions based on sensor inputs or remote commands, utilizing feedback from encoders to adjust robot movement accurately.
Sensor Integration: Use infrared and ultrasonic sensors to detect obstacles, aiding in autonomous navigation.
User Input: Leverage joystick inputs for manual control of the robot's movement.
Vision Processing: Utilize the camera for visual tasks like navigation markers detection, object recognition, and environment mapping.

### interfaceRPRK.ino Sketch (Arduino)

---

## Scripts

### Left Wall Follower

### Keyboard Control With Avoidance

## PID Wheel Control

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
