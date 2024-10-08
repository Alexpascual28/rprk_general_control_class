# -*- coding: utf-8 -*-
"""
Created on Thu Sep 3 2024

@author: Alejandro Pascual San Roman (bdh532)
@organisation: Department of Physics, Engineering and Technology. University of York
@title: RPRK class (formerly Turtlebot class)

@version: v4

"""

from ARBPi import *
import math
import time
import numpy as np
import threading
import ctypes
import csv
import os

# Import the necessary packages
import picamera
import picamera.array
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

class RPRK:
    def __init__(self):
        # Setup the ARB functions
        print("Setting up ARB")
        ARBPiSetup(SERIAL)
        
        self.camera = self.Camera()
        self.infrared = self.InfraredSensor()
        self.ultrasound = self.Ultrasonic()
        self.joystick = self.Joystick()
        self.serial = self.Serial()
        self.motors = self.Motors(self.serial)
            
    class Motors:
        def __init__(self, serial):
            # Serial registers
            # PID
            self.REG_RECEIVE_KP_A = 30 # Receives proportional multiplier of motor A
            self.REG_RECEIVE_KP_A_DEC = 31 # Receives proportional multiplier of motor A (decimal part)

            self.REG_RECEIVE_KP_B = 32 # Receives proportional multiplier of motor B
            self.REG_RECEIVE_KP_B_DEC = 33 # Receives proportional multiplier of motor B (decimal part)

            self.REG_RECEIVE_KI_A = 34 # Receives integral multiplier of motor A
            self.REG_RECEIVE_KI_A_DEC = 35 # Receives integral multiplier of motor A (decimal part)

            self.REG_RECEIVE_KI_B = 36 # Receives integral multiplier of motor B
            self.REG_RECEIVE_KI_B_DEC = 37 # Receives integral multiplier of motor B (decimal part)

            self.REG_RECEIVE_KD_A = 38 # Receives derivative multiplier of motor A
            self.REG_RECEIVE_KD_A_DEC = 39 # Receives derivative multiplier of motor A (decimal part)

            self.REG_RECEIVE_KD_B = 40 # Receives derivative multiplier of motor B
            self.REG_RECEIVE_KD_B_DEC = 41 # Receives derivative multiplier of motor B (decimal part)

            self.REG_RECEIVE_SETPOINT_A = 42 # PID Setpoint of wheel A
            self.REG_RECEIVE_SETPOINT_A_DEC = 43 # PID Setpoint of wheel A (decimal part)

            self.REG_RECEIVE_SETPOINT_B = 44 # PID Setpoint of wheel B
            self.REG_RECEIVE_SETPOINT_B_DEC = 45 # PID Setpoint of wheel B (decimal part)

            self.REG_RECEIVE_TIMECHANGE = 46 # Millisecond delay between PID loop iterations
            self.REG_RECEIVE_STEP_DISTANCE = 47 # Distance per step
            self.REG_RECEIVE_DISTANCE_BETWEEN_WHEELS = 48 # Distance between wheels in centimetres
            self.REG_RECEIVE_WHEEL_DIAMETER = 49 # Wheel diameter
            self.REG_RECEIVE_WHEEL_DIAMETER_DEC = 50 # Wheel diameter (decimal part)

            # POSE
            self.REG_SEND_POSE_X = 51 # Robot pose x
            self.REG_SEND_POSE_X_DEC = 52 #Robot pose x (decimal part)

            self.REG_SEND_POSE_Y = 53 # Robot pose y
            self.REG_SEND_POSE_Y_DEC = 54 # Robot pose y (decimal part)

            self.REG_SEND_POSE_W = 55 # Robot pose w
            self.REG_SEND_POSE_W_DEC = 56 # Robot pose w (decimal part)

            # DRIVE DATA
            self.REG_RECEIVE_GOAL_MARGIN = 57 # PID goal margin
            self.REG_RECEIVE_GOAL_MARGIN_DEC = 58 # PID goal margin (decimal part)
            self.REG_RECEIVE_PID_SIGNAL = 59 # PID Data signal
            self.REG_SEND_EXIT_CODE = 60 # Send exit code drive control data

            self.REG_RECEIVE_CONTROL_MODE = 61 # Receives control mode input

            self.REG_RECEIVE_SPEED_DATA = 62 # Receive speed data
            self.REG_RECEIVE_MSG_DRIVE = 63 # Receive drive control data
            self.REG_SEND_MSG_DRIVE = 64 # Send drive control data

            # HANDSHAKE

            self.REG_SEND_CONFIRM = 65 # Send handshake signal
            self.REG_RECEIVE_ACK = 66 # Receive handshake signal

            # PID CONFIGURATION VARIABLES
            # Instatiate the RPRK class
            self.serial = serial
            self.current_pose = [0, 0, 0] # x, y, w

            # Motor A tunings
            self.kpA = 30
            self.kiA = 0.1
            self.kdA = 0.5
            
            # Motor B tunings
            self.kpB = 30
            self.kiB = 0.1
            self.kdB = 0.5

            self.goal_margin = 0.02 # Goal margin for PID control
            self.timechange = 1 # 1 millisecond delay per PID compute
            self.step_distance_cm = 5 # Distance per step
            
            # ODOMETRY
            self.distance_between_wheels = 20 # In cm
            self.wheel_diameter = 4.75

            # HANDSHAKE VARIABLES
            self.timeout_ms = 5000 # Timeout in milliseconds

            # Set all initial values
            self.set_pid_tunings("A", self.kpA, self.kiA, self.kdA);
            self.set_pid_tunings("B", self.kpB, self.kiB, self.kdB);
            self.set_timechange(self.timechange);
            self.set_step_distance(self.step_distance_cm);
            self.set_distance_between_wheels(self.distance_between_wheels);
            self.set_wheel_diameter(self.wheel_diameter);
            self.set_goal_margin(self.goal_margin);
            time.sleep(0.5)
            self.set_control_mode("PID");
        
        # PROTO-HANDSHAKE COMMUNICATION METHODS

        def send_data(self, data, register):
            current_time = 0
            exit_loop = False

            if(len(register) == 1):
                putRegister(register[0], data)
                print(f"Data sent: {data} to register {register[0]}")
            elif(len(register) == 2):
                self.serial.send_fractional_number(data, register[0], register[1])
                print(f"Data sent: {data} to registers {register[0]} and {register[1]}")
            else:
                raise Exception("Incorrect number of registers")
            
            while(exit_loop == False):
                print("Waiting for confirmation")

                confirm_signal = getRegister(self.REG_SEND_CONFIRM)

                print(f"Confirm signal: {confirm_signal}")

                current_time += self.timechange

                if(confirm_signal == 1 or confirm_signal == 2):
                    putRegister(self.REG_RECEIVE_ACK, 1)

                    while(exit_loop == False):
                        confirm_signal = getRegister(self.REG_SEND_CONFIRM)

                        print(f"ACK Confirm signal: {confirm_signal}")

                        current_time += self.timechange

                        if(confirm_signal == 4 or confirm_signal == 5):
                            print("Data received")
                            exit_loop = True

                        elif(confirm_signal == 3):
                            putRegister(self.REG_SEND_CONFIRM, 0)
                            exit_loop = True
                            raise Exception("Arduino timed out. ACK signal not received.")

                        elif(current_time >= self.timeout_ms):
                            exit_loop = True
                            raise TimeoutError("Timeout after confirmation")
                            
                        # time.sleep(self.timechange/1000)

                elif(confirm_signal == 3):
                    putRegister(self.REG_SEND_CONFIRM, 0)
                    exit_loop = True
                    print("Data received, but confirmation failed.")

                elif(confirm_signal == 4 or confirm_signal == 5):
                    putRegister(self.REG_SEND_CONFIRM, 0)
                    exit_loop = True
                    print("Value already set.")

                elif(current_time >= self.timeout_ms):
                    exit_loop = True
                    raise TimeoutError("Timeout")
                    
                # time.sleep(self.timechange/1000)

        # CONFIGURATION METHODS

        # Method to set control mode
        def set_control_mode(self, control_mode):
            control_modes = {"PID": 1, "SPEED": 0}

            if control_mode in control_modes:
                # Send data
                self.send_data(control_modes.get(control_mode), [self.REG_RECEIVE_CONTROL_MODE,])
                print(f"Control mode: {control_mode}")
            else:
                raise Exception("Incorrect mode.")

        # Method to set time per PID loop iteration in milliseconds
        def set_timechange(self, timechange):
            # Send data
            self.send_data(timechange, [self.REG_RECEIVE_TIMECHANGE,])
            print(f"PID iteration time: {timechange} ms")

        # Method to set distance per step
        def set_step_distance(self, step_distance):
            # Send data
            self.send_data(step_distance, [self.REG_RECEIVE_STEP_DISTANCE,])
            self.step_distance_cm = step_distance
            print(f"Step distance: {step_distance} cm")

        # Method to set distance between wheels
        def set_distance_between_wheels(self, distance_between_wheels):
            # Send data
            self.send_data(distance_between_wheels, [self.REG_RECEIVE_DISTANCE_BETWEEN_WHEELS,])
            self.distance_between_wheels = distance_between_wheels
            print(f"Distance between wheels: {distance_between_wheels} cm")

        # Method to set wheel diameter
        def set_wheel_diameter(self, wheel_diameter):
            # Send data
            self.send_data(wheel_diameter, [self.REG_RECEIVE_WHEEL_DIAMETER, self.REG_RECEIVE_WHEEL_DIAMETER_DEC])
            self.wheel_diameter = wheel_diameter
            print(f"Wheel diameter: {wheel_diameter} cm")

        # Method to set goal margin
        def set_goal_margin(self, goal_margin):
            # Send data
            self.send_data(goal_margin, [self.REG_RECEIVE_GOAL_MARGIN, self.REG_RECEIVE_GOAL_MARGIN_DEC])
            self.goal_margin = goal_margin
            print(f"Goal margin: {goal_margin}")
                
        # Method to set individual PID tuning
        def set_pid_tuning(self, motor, tuning, value):
            tuning_registers = {"A": {"KP": (self.REG_RECEIVE_KP_A, self.REG_RECEIVE_KP_A_DEC), "KI": (self.REG_RECEIVE_KI_A, self.REG_RECEIVE_KI_A_DEC), "KD": (self.REG_RECEIVE_KD_A, self.REG_RECEIVE_KD_A_DEC)},
                                "B": {"KP": (self.REG_RECEIVE_KP_B, self.REG_RECEIVE_KP_B_DEC), "KI": (self.REG_RECEIVE_KI_B, self.REG_RECEIVE_KI_B_DEC), "KD": (self.REG_RECEIVE_KD_B, self.REG_RECEIVE_KD_B_DEC)}}
            
            if motor in tuning_registers:
                if tuning in tuning_registers.get(motor):
                    # Send data
                    self.send_data(value, [tuning_registers.get(motor).get(tuning)[0], tuning_registers.get(motor).get(tuning)[1]])
                    print(f"PID tuning {tuning} for motor {motor} set to: {value}")
                else:
                    raise Exception("Incorrect tuning.")
            else:
                raise Exception("Incorrect motor.")

         # Method to set PID tunings
        def set_pid_tunings(self, motor, kp, ki, kd):
            print(f"Setting PID tunings for motor {motor}: KP: {kp}, KI: {ki}, KD: {kd}")
            self.set_pid_tuning(motor, "KP", kp)
            self.set_pid_tuning(motor, "KI", ki)
            self.set_pid_tuning(motor, "KD", kd)

        # PID CONTROL METHODS

        # Method to set PID setpoint
        def set_pid_setpoint(self, motor, value):
            setpoint_registers = {"A": [self.REG_RECEIVE_SETPOINT_A, self.REG_RECEIVE_SETPOINT_A_DEC],
                                  "B": [self.REG_RECEIVE_SETPOINT_B, self.REG_RECEIVE_SETPOINT_B_DEC]}
            
            if motor in setpoint_registers:
                r1, r2 = setpoint_registers.get(motor);
                self.send_data(value, [r1, r2])
                print(f"PID setpoint for motor {motor} set to: {value}")
            else:
                raise Exception("Incorrect motor.")

        # Method to get robot pose
        def get_pose(self):
            x = self.serial.read_fractional_number(self.REG_SEND_POSE_X, self.REG_SEND_POSE_X_DEC)
            y = self.serial.read_fractional_number(self.REG_SEND_POSE_Y, self.REG_SEND_POSE_Y_DEC)
            w = self.serial.read_fractional_number(self.REG_SEND_POSE_W, self.REG_SEND_POSE_W_DEC)
            
            self.current_pose = [x, y, w]
            return self.current_pose
        
        # Method to get exit code
        def get_exit_code(self):
            return getRegister(self.REG_SEND_EXIT_CODE)
        
        # Method to send stop signal
        def send_PID_signal(self, signal):
            possible_signals = {"continue": 0, "stop": 1, "direction": 2}
            self.send_data(possible_signals.get(signal), [self.REG_RECEIVE_PID_SIGNAL,])
            print(f"PID {signal} signal sent.")

        # Method to move the robot in a given direction for a given duration
        def move_robot(self, direction, duration):
            current_time = 0
            self.send_PID_signal("direction")

            while current_time < duration:
                self.change_direction(direction)
                time.sleep(self.timechange/1000)
                current_time += self.timechange

            self.change_direction("stop")

        # Method to move the robot straight for a given distance
        def advance_robot(self, distance):
            exit_signal = 0
            self.send_PID_signal("continue")
            self.set_pid_setpoint("A", distance)
            self.set_pid_setpoint("B", distance)

            while exit_signal != -1:
                exit_signal = getRegister(self.REG_RECEIVE_PID_SIGNAL)
                time.sleep(0.1)
            
            print(f"Exit signal: {exit_signal}")

        # Method to rotate the robot by a given angle
        def rotate_robot(self, angle_radians):
            exit_code = 0
            self.send_PID_signal("continue")

            distance = angle_radians * self.distance_between_wheels / 2

            self.set_pid_setpoint("A", distance)
            self.set_pid_setpoint("B", -distance)

            while exit_code < 2:
                exit_code = self.get_exit_code()
                time.sleep(self.timechange/1000)

            print(f"Exit code: {exit_code}")

        # SPEED CONTROL METHODS
            
        # Changes the robot direction based on global direction commands
        def change_direction(self, direction):
            possible_directions = {"forward": 1, "backward": 2, "left": 3, "right": 4, "stop": 5}
            
            if direction in possible_directions:
                # Send data
                putRegister(self.REG_RECEIVE_MSG_DRIVE, possible_directions.get(direction))
                print(f"Moving {direction}")
            else:
                raise Exception("Incorrect direction.")
                
        # Changes the robot speed based on the given speed level
        def set_robot_speed_by_level(self, speed_level):
            # If the user input is 0-9
            if speed_level in range(10):
                # Send data
                putRegister(self.REG_RECEIVE_SPEED_DATA, speed_level)
                print(f"Setting speed level to {speed_level}")
            else:
                raise Exception("Incorrect speed level.")

    class Camera:
        def __init__(self):
            # Initialise the camera and create a reference to it
            self.camera = picamera.PiCamera()
            self.camera.rotation = 180
            self.camera.resolution = (640, 480) # HD: (1280, 960)
            self.camera.framerate = 32
            self.rawCapture = picamera.array.PiRGBArray(self.camera, size=self.camera.resolution)
            
            # Allow the camera time to warm up
            time.sleep(0.1)
            
            # Setup aruco dictionary and parameters
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.aruco_parameters = aruco.DetectorParameters_create()
            
            # Create counter for FPS
            self.frame_count = 0
            self.start_time = time.time()

            # Create members to store current image data
            ctypes.CDLL('libX11.so.6').XInitThreads()
            self.camera_thread = threading.Thread(target=self.capture_frame_continuous, args=(False,))
            self.current_image = None

            # Create CSV file with default hsv values if none exists already, and read those values to possible colours
            self.hsv_filename = 'hsv_colour_values.csv'
            self.current_hsv_values = self.read_hsv_values(self.hsv_filename)

        # Start continuous image acquisition
        def start_image_acquisition(self, show_feed = True):
            self.camera_thread = threading.Thread(target=self.capture_frame_continuous, args=(show_feed,))
            self.camera_thread.start()

        # Returns current image
        def get_current_image(self):
            return self.current_image
        
        def capture_frame_continuous(self, show_frame = True):
            try:
                # Capture frames from the camera
                for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                    self.current_image = frame.array
                    
                    # Show the frame if show_frame is true
                    if show_frame == True: cv2.imshow("Current Frame", self.current_image)

                    # Clear the stream in preparation for the next frame
                    self.rawCapture.truncate(0)

                    # if the `q` key was pressed, break from the loop
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            finally:
                print("Closing camera")
                self.camera.close()

        def show_image(self, frame_name, image):
            cv2.imshow(frame_name, image)
        
        def detect_aruco(self, image, frame_name="Aruco Frame", show_frame = True):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
            
            frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
           
            self.frame_count += 1
            average_fps = self.frame_count / ( time.time() - self.start_time )
            cv2.putText(frame_markers,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

            # Show the frame
            if show_frame == True: cv2.imshow(frame_name, frame_markers)

            return frame_markers, corners, ids
        
        def detect_blobs(self, image, colour_name, frame_name="Blobs Frame", show_frame = True):
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            if colour_name in self.current_hsv_values:
                hsv_min = tuple(self.current_hsv_values.get(colour_name)[0])
                hsv_max = tuple(self.current_hsv_values.get(colour_name)[1])

                mask=cv2.inRange(hsv,hsv_min,hsv_max)

                params = cv2.SimpleBlobDetector_Params()
                params.thresholdStep = 255
                params.minRepeatability = 1
                params.blobColor = 255
                params.filterByInertia = False
                params.filterByConvexity = False

                #To just detect colour:
                #params.filterByArea = False

                #To detect colour and area:
                params.filterByArea = True
                params.minArea = 100
                params.maxArea = 80000 # 20000 if HD

                detector = cv2.SimpleBlobDetector_create(params)
                keypoints = detector.detect(mask)
                kp_image = cv2.drawKeypoints(mask,keypoints,None,color=(0,0,255),flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                self.frame_count += 1
                average_fps = self.frame_count / ( time.time() - self.start_time )
                cv2.putText(kp_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

                #masked_image = cv2.bitwise_and(image,image,mask=mask)
                # Show the frame
                if show_frame == True: cv2.imshow(frame_name, kp_image)

                return kp_image
            else:
                print("Incorrect colour value.")
                return None

                # blueMin= (105,80,45)
                # blueMax= (155,255,230)

        def detect_colour(self, image, colour_name, frame_name="colour frame", image_format="hsv", show_frame = True):
            blurred_image = self.blurring(image)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            if colour_name in self.current_hsv_values:
                hsv_min = tuple(self.current_hsv_values.get(colour_name)[0])
                hsv_max = tuple(self.current_hsv_values.get(colour_name)[1])

                mask=cv2.inRange(hsv,hsv_min,hsv_max)
                masked_image = cv2.bitwise_and(image,image,mask=mask)

                self.frame_count += 1
                average_fps = self.frame_count / ( time.time() - self.start_time )
                cv2.putText(masked_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

                # Show the frame
                if show_frame == True:
                    if frame_name == "colour frame": cv2.imshow(f"{colour_name} {frame_name}", masked_image)
                    else: cv2.imshow(f"{frame_name}", masked_image)

                return mask, masked_image
            else:
                print("Incorrect colour value.")
                return None

            # print(hsv_min)
            # print(hsv_max)

            # blueMin= (105,80,45)
            # blueMax= (155,255,230)
                
        def detect_shapes(self, mask, shape_name, frame_name="Shape Frame", show_frame = False):
            possible_shapes = {'triangle': 3, 'rectangle': 4, 'star': 10, 'circle': 11}

            if shape_name in possible_shapes:
                shapes = []

                mask = self.dilating(mask)
                mask = self.opening(mask)

                mask = self.eroding(mask)
                mask = self.closing(mask)
                
                mask = self.canny_edge_detection(mask)
                mask = self.dilating(mask)

                contours, h = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE)
                contours.sort(key = len)

                shape_counter = 0

                for contour in contours[-3:]:
                    #Amount of edges
                    approx = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)

                    #Center locations
                    M = cv2.moments(contour)
                    if M['m00'] == 0.0:
                        continue
                    centroid_x = int(M['m10']/M['m00'])
                    centroid_y = int(M['m01']/M['m00'])

                    if (len(approx) == possible_shapes.get(shape_name)) or (len(approx) >= 11 and shape_name == 'circle'):
                        shape = [f"{shape_name} {shape_counter}", contour, centroid_x, centroid_y, len(approx)]
                        shapes.append(shape)
                        shape_counter += 1

                return shapes
            
            else:
                print("Incorrect shape value.")
                return None
        
        def closing(self, mask):
            kernel = np.ones((7,7),np.uint8) 
            closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            return closing

        def opening(self, mask):
            kernel = np.ones((6,6),np.uint8)
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            return opening

        def blurring(self, mask):
            blur = cv2.GaussianBlur(mask,(5,5),0)
            return blur

        def eroding(self, mask):
            kernel = np.ones((5,5),np.uint8)
            erosion = cv2.erode(mask, kernel, iterations = 1)
            return erosion

        def dilating(self, mask):
            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(mask, kernel, iterations = 1)
            return dilation

        def canny_edge_detection(self, mask):
            edges = cv2.Canny(mask,100,200)
            return edges

        # Creates new csv file and populates it with default values if none exist with the same name
        def create_hsv_csv_file(self, filename):
            current_directory = os.getcwd()

            default_hsv_values = {"red": [(172,120,0), (180,255,255)], # red hsv range: low(179,0,0), high(180,255,255) // red,172,120,0,180,255,255
                                "blue": [(109,116,47),(115,255,100)], # blue hsv range: low(109,116,47), high(118,255,255) // blue,109,116,47,115,255,100
                                "green":[(65, 61, 0),(95, 255, 255)], # green hsv range: low(65, 61, 0), high(79, 255, 255) // green,65,61,0,95,255,255
                                "lilac":[(116, 60, 66),(152, 255, 255)],} # lilac hsv range: low(116, 60, 66), high(152, 255, 255) // lilac,116,60,66,152,255,255

            if not os.path.isfile(current_directory + '/' + filename):
                self.write_hsv_values(filename, default_hsv_values)
                        
        def read_hsv_values(self, filename):
            self.create_hsv_csv_file(filename) # Create new file if it does not exist

            with open(filename, mode='r') as file:
                reader = csv.reader(file)
                hsv_values = {rows[0]:[[int(rows[1]), int(rows[2]), int(rows[3])], [int(rows[4]), int(rows[5]), int(rows[6])]] for rows in reader}

            return hsv_values
        
        def write_hsv_values(self, filename, hsv_values):
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                for colour in hsv_values:
                    writer.writerow([colour, hsv_values[colour][0][0], hsv_values[colour][0][1], hsv_values[colour][0][2],
                                             hsv_values[colour][1][0], hsv_values[colour][1][1], hsv_values[colour][1][2]])
        
        def change_hsv_values(self, colour, parameter, range, value):
            self.current_hsv_values = self.read_hsv_values(self.hsv_filename)

            colours = ["red", "blue", "green", "lilac"]
            parameters = ["hue", "saturation", "value"]
            ranges = ["low", "high"]

            if colour in colours and parameter in parameters and range in ranges:
                self.current_hsv_values[colour][ranges.index(range)][parameters.index(parameter)] = value
                self.write_hsv_values(self.hsv_filename, self.current_hsv_values)
            else:
                print("Invalid input")
            
        def generate_aruco_tags(self, filename):
            filename = f"{filename}.pdf"
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

            fig = plt.figure()
            nx = 3
            ny = 2
            for i in range(1, nx*ny+1):
                ax = fig.add_subplot(ny,nx, i)
                img = aruco.drawMarker(aruco_dict,i, 700)
                plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
                ax.axis("off")

            plt.savefig(filename)
            plt.show()

        # Capture a single frame from the camera
        def capture_frame(self, show_frame = True):
            image = None
            frame = self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)

            if frame is not None:
                image = frame.array
                
                # Show the frame
                if show_frame == True: cv2.imshow("Frame", image)

            # Clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            return image
            
    class Ultrasonic:
        def __init__(self): 
            self.REG_SEND_DATA_ULTRASOUND_1 = 20 # Ultrasound 1 send data 
            self.REG_SEND_DATA_ULTRASOUND_2 = 21 # Ultrasound 2 send data
        
        # Get data from the appropiate register based on target sensor
        def get_ultrasound_distance(self, sensor): 
            possible_sensors = {"left": self.REG_SEND_DATA_ULTRASOUND_1,
                                "right": self.REG_SEND_DATA_ULTRASOUND_2}
            
            if sensor in possible_sensors:
                distance = getRegister(possible_sensors.get(sensor))
                distance = np.uint8(distance)
                return distance
            else:
                raise Exception(f"Incorrect sensor name: {sensor}")
                                
    class Joystick:
        def __init__(self):
            self.REG_SEND_MSG_JOYSTICK = 70 # Joystick send messages
            self.REG_RECEIVE_MSG_JOYSTICK = 71 # Joystic receive messages
            self.REG_SEND_DATA_JOYSTICK = 72 # Joystick send data (direction)
        
        # Get current joystick direction
        def get_joystick_direction(self):
            direction_data = getRegister(self.REG_SEND_DATA_JOYSTICK)
            
            possible_directions = {0: "stop", 1: "forward", 2:"backward", 
                                   3:"left", 4:"right", 5:"stop"}
            
            if direction_data in possible_directions:
                direction = possible_directions.get(direction_data)
                print(f"The joystick is moving {direction}")
                return direction
            else:
                raise Exception(f"Unusual value read from the register: {direction_data}")
            
    class InfraredSensor:
        def __init__(self):
            # I2C MUX communication 
            self.REG_SEND_IR_1 = 10 # Serial register to send the IR data to IR 1
            self.REG_SEND_IR_2 = 11 # Serial register to send the IR data to IR 2
            self.REG_SEND_IR_3 = 12 # Serial register to send the IR data to IR 3
            self.REG_SEND_IR_4 = 13 # Serial register to send the IR data to IR 4
        
            # Get data from the register
        def get_infrared_distance(self, sensor):
            possible_sensors = {"left": self.REG_SEND_IR_1, "right": self.REG_SEND_IR_4}

            return getRegister(possible_sensors.get(sensor))

    class Serial:
        # Reads two values from separate registers and turns them into a 16bit signed integer
        def read_16bit_number(self, register1, register2):
            first_part = getRegister(register2)
            second_part = getRegister(register1)
            
            result = (first_part << 7) + second_part # Shifts first value by seven bits and add the second value
            return result
        
        # Sends a 16 bit number by separating it into two numbers and sending them to different registers
        def send_16bit_number(self, number, register1, register2):
            first_part = number >> 7
            second_part = number - (first_part << 7)
            
            putRegister(register1, second_part)
            putRegister(register2, first_part)

        # Reads two values from separate registers and turns them into a 2-fractional decimal number
        def read_fractional_number(self, register1, register2):
            wholePart = getRegister(register1)
            fractPart = getRegister(register2)
            
            resultFrac = wholePart + fractPart/100
            return resultFrac
        
        def send_fractional_number(self, number, register1, register2):
            wholePart = int(number)
            fractPart = int(number*100 - wholePart*100)

            putRegister(register1, wholePart)
            putRegister(register2, fractPart)