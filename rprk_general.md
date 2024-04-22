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

<code>► INSERT-TEXT-HERE</code>

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

3. **Open a connection to the Raspberry Pi** using PuTTY or directly through HDMI. (Windows instructions):
   1. Connect to the Raspberry Pi with your laptop using the serial **USB to UART HAT** and a USB cable.
   2. Check what *COM* port the device is connected to using "Device Manager"
   3. Establish a `Serial` connection with PuTTY using the device *COM* port and baud rate 115200.
   
   Alternatively, you can connect a screen and keyboard directly to the Raspberry Pi to access the terminal directly.

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
      * Have [XMing](http://www.straightrunning.com/XmingNotes/) installed in your device.
      * Execute XMing before establishing a connection. It will run in the background.
      * In PuTTY, go to **Connections** > **SSH** > **X11** and check the box that says *'Enable X-11 forwarding'*.
   4. If you wish, save the session under your preferred name by going to **Session** > **"Load, save or delete a stored session"**. Write your session name under *Saved Sessions* and click **"Save"**.
   5. Click **"Open"** at the bottom-right of the window.
   6. Login using your login details for the Raspberry Pi in CLI. For the lab RPRK devices, the details are the following:
      * **Username:** *pi*
      * **Password:** *raspberry*

6. **View, add and modify files using WinSCP** (Windows instructions).
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

**Running the Code**

To run Arduino files, upload the respective `.ino` sketch files to the Arduino Nano using the Arduino IDE. There are two main `.ino` files that can be 

For the Raspberry Pi, navigate to the specific target file and run the Python scripts through the terminal:

```bash
python3 <script_name>.py
```

Ensure that the `ARBPi` file and the `RPRK` files or other dependencies are located in the same folder when running a script that requires their functions.

Ensure that the Raspberry Pi and ARB are connected via UART, as the scripts and sketches often communicate over this channel.

---

##  Project Roadmap

- [X] `► INSERT-TASK-1`
- [ ] `► INSERT-TASK-2`
- [ ] `► ...`

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
