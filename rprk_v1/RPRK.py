# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 2024

@author: Alejandro Pascual San Roman (bdh532)
@organisation: Department of Physics, Engineering and Technology. University of York
@title: RPRK class (formerly Turtlebot class)

@version: v3

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
        
        self.initial_pose = [0, 0, 0] #  x, y, w(orientation)
        
        self.motors = self.Motors(self, self.initial_pose)
        self.camera = self.Camera()
        self.infrared = self.InfraredSensor()
        self.ultrasound = self.Ultrasonic()
        self.joystick = self.Joystick()
        
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
        wholePart = int(number);
        fractPart = number*100 - wholePart*100;

        putRegister(register1, wholePart);
        putRegister(register2, fractPart);
            
    class Motors:
        def __init__(self, rprk, initial_pose):
            self.distance_between_wheels = 21
            
            self.distance_step = 0.4 # The robot will move x centimetres per step
            self.distance_margin = 0.2 # The robot will reach its destination if it is within x centimetres of target distance
            self.linear_speed = 5 # The robot will move at x centimetres per second initially, when moving forwards or backwards
            
            self.speed_a = 5
            self.speed_b = 5
            
            self.rotational_step = math.pi/16 # The robot will rotate x radians per rotation
            self.rotational_margin = math.pi/128 # The robot will reach its target orientation if it is within x radians
            self.rotation_speed = 10 # The wheels will move x centimetres per second, when the robot is rotating
            
            self.rprk = rprk
            self.current_pose = initial_pose # x, y, w
            
            # Serial registers
            self.REG_RECEIVE_DIR_MOTOR_A = 30 # Motor A receive direction
            self.REG_RECEIVE_PWM_MOTOR_A = 31 # Motor A receive PWM
            self.REG_SEND_DATA_MOTOR_A = 32 # Motor A send data (current direction and speed)

            self.REG_RECEIVE_DIR_MOTOR_B = 33 # Motor B receive direction
            self.REG_RECEIVE_PWM_MOTOR_B = 34 # Motor B receive PWM
            self.REG_SEND_DATA_MOTOR_B = 35 # Motor B send data (current direction and speed)

            self.REG_SEND_DATA_ENCODER_A_1 = 36 # Encoder A send messages (Rasperry receives)
            self.REG_SEND_DATA_ENCODER_A_2 = 37 # Encoder A send data (Raspberry recieves)

            self.REG_SEND_DATA_ENCODER_B_1 = 38 # Encoder B send messages (Raspberry receives)
            self.REG_SEND_DATA_ENCODER_B_2 = 39 # Encoder B send data (Raspberry receives)

            self.REG_RECEIVE_SPEED_DATA = 40 # Arduino receives speed data (Raspberry sends)
            self.REG_RECEIVE_MSG_DRIVE = 41 # Arduino receives drive control data (Raspberry sends)
            self.REG_SEND_MSG_DRIVE = 42 # Arduino sends drive control data (current state) (Raspberry receives)

            self.REG_SEND_DISTANCE_A = 43 # Calculated distance in wheel A
            self.REG_SEND_DISTANCE_A_DEC = 44 # Calculated distance in wheel A (decimal part)

            self.REG_SEND_DISTANCE_B = 45 # Calculated distance in wheel B
            self.REG_SEND_DISTANCE_B_DEC = 46 # Calculated distance in wheel B (decimal part)

            self.REG_SEND_SPEED_A = 47 # Calculated speed of wheel A
            self.REG_SEND_SPEED_A_DEC = 48 # Calculated speed of wheel A (decimal part)

            self.REG_SEND_SPEED_B = 49 # Calculated speed of wheel B
            self.REG_SEND_SPEED_B_DEC = 50 # Calculated speed of wheel B (decimal part)
            
            self.REG_RECEIVE_RESET_ENCODER_A = 51 # Resets absolute steps of encoder A
            self.REG_RECEIVE_RESET_ENCODER_B = 52 # Resets absolute steps of encoder B

            self.REG_RECEIVE_CONTROL_MODE = 53 # Receives control mode input

            self.REG_RECEIVE_KP_A = 54 # Receives proportional multiplier of motor A
            self.REG_RECEIVE_KP_B = 55 # Receives proportional multiplier of motor B

            self.REG_RECEIVE_KI_A = 56 # Receives integral multiplier of motor A
            self.REG_RECEIVE_KI_B = 57 # Receives integral multiplier of motor B

            self.REG_RECEIVE_KD_A = 58 # Receives derivative multiplier of motor A
            self.REG_RECEIVE_KD_B = 59 # Receives derivative multiplier of motor B

            self.REG_RECEIVE_SETPOINT_A = 70 # PID Setpoint of wheel A
            self.REG_RECEIVE_SETPOINT_A_DEC = 71 # PID Setpoint of wheel A (decimal part)

            self.REG_RECEIVE_SETPOINT_B = 72 # PID Setpoint of wheel B
            self.REG_RECEIVE_SETPOINT_B_DEC = 73 # PID Setpoint of wheel B (decimal part)
            
        # Changes the robot direction based on global direction commands
        def change_direction(self, direction):
            possible_directions = {"forward": 1, "backward": 2, "left": 3, "right": 4, "stop": 5}
            
            if direction in possible_directions:
                # Send data
                putRegister(self.REG_RECEIVE_MSG_DRIVE, possible_directions.get(direction))
                print(f"Moving {direction}")
            else:
                print("Incorrect direction.")
                
        # Changes the robot speed based on the given speed level
        def set_robot_speed_by_level(self, speed_level):
            # If the user input is 0-9
            if speed_level in range(10):
                # Send data
                putRegister(self.REG_RECEIVE_SPEED_DATA, speed_level)
                print(f"Setting speed level to {speed_level}")
            else:
                print("Incorrect speed level.")
                
        # Changes the wheel speed based on a given speed in centimetres per second
        def set_wheel_speed(self, wheel, speed_cm_s):
            # Identifies which registers to send to based on input
            possible_wheels = {"A": [self.REG_RECEIVE_PWM_MOTOR_A, 140, 8.356667],
                               "B": [self.REG_RECEIVE_PWM_MOTOR_B, 140, 8.042222]}
            if wheel in possible_wheels:
                # Calculates PWM signal from given metric speed in centimetres per second (cm/s)
                average_pwm_signal = possible_wheels.get(wheel)[1] # Measured average PWM
                average_speed_cm_s = possible_wheels.get(wheel)[2] # Measured average speed
                speed_pwm = int((average_pwm_signal/average_speed_cm_s)*speed_cm_s); # Converts to int
                
                # Adjusts value
                if speed_pwm > 255:
                    speed_pwm = 255
                elif speed_pwm < 0:
                    speed_pwm = 0
                
                # Sends value to the appropiate register
                putRegister(possible_wheels.get(wheel)[0], speed_pwm)
                print(f"Setting speed of wheel {wheel} to {speed_cm_s} cm/s ({speed_pwm} PWM)")
            else:
                print("Incorrect wheel input.")
        
        # Changes the wheel direction based on a given direction command
        def set_wheel_direction(self, wheel, direction):
            # Identifies which registers to send to based on input
            possible_wheels = {"A": self.REG_RECEIVE_DIR_MOTOR_A,
                               "B": self.REG_RECEIVE_DIR_MOTOR_B}
            
            possible_directions = {"CW": 0, "CCW": 1}
            
            if wheel in possible_wheels:
                if direction in possible_directions:
                    # Sends appropiate value to the appropiate register
                    putRegister(possible_wheels.get(wheel), possible_directions.get(direction))
                    print(f"Setting wheel {wheel} direction to {direction}")
                else:
                    print("Incorrect wheel direction")
            else:
                print("Incorrect wheel input.")
        
        # Sets speed of both wheels simultaneously
        def set_robot_speed(self, speed_a, speed_b=None):
            if speed_b == None:
                self.set_wheel_speed("A", speed_a);
                self.set_wheel_speed("B", speed_a);
            else:
                self.set_wheel_speed("A", speed_a);
                self.set_wheel_speed("B", speed_b);
            
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
        
        # Reads current travelled distance from the encoder registers
        def get_current_distance(self, encoder):
            possible_encoders = {"A": [self.REG_SEND_DISTANCE_A, self.REG_SEND_DISTANCE_A_DEC],
                                 "B": [self.REG_SEND_DISTANCE_B, self.REG_SEND_DISTANCE_B_DEC]}
            
            if encoder in possible_encoders:
                r1, r2 = possible_encoders.get(encoder)
                distance = self.rprk.read_fractional_number(r1, r2)
                return distance
            else:
                print("Incorrect encoder.")
        
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
                
        # Resets encoders
        def reset_encoder(self, encoder):
            possible_encoders = {"A": self.REG_RECEIVE_RESET_ENCODER_A,
                                 "B": self.REG_RECEIVE_RESET_ENCODER_B}
            
            if encoder in possible_encoders:
                putRegister(possible_encoders.get(encoder), 1)
            else:
                print("Incorrect encoder.")
                
        # Calculates the next pose based on calculated travelled distance of each wheel and stores it in self.current_pose
        def calculate_current_pose(self, distance_a, distance_b):
            # With my own calculations. This is all geometrically correct, but may cause problems if the distances are similar, as the radius value would approach infinity.
            # radius_b = (distance_b * self.distance_between_wheels)/(distance_a - distance_b) 
            # angle_change = distance_b/radius_b
            # radius = radius_b + self.distance_between_wheels/2
            # distance_change = 2 * radius * math.sin(angle_change/2)
            
            # Calculations based on https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
            angle_change = (distance_b - distance_a)/self.distance_between_wheels
            distance_change = (distance_a + distance_b)/2 # Since the movement is small, it assumes the circular distance is equal to the linear distance
            
            self.current_pose[0] = self.current_pose[0] + distance_change * math.cos(self.current_pose[2]+ (angle_change/2))
            self.current_pose[1] = self.current_pose[1] + distance_change * math.sin(self.current_pose[2]+ (angle_change/2))
            self.current_pose[2] = self.current_pose[2] + angle_change
            
            return distance_change, angle_change
      
        # Calculate distance travelled and speed
        def calculate_speed_cm_s(self, distance_cm, time_difference_s):
          speed = distance_cm/time_difference_s; # Speed in cm/second
          return speed
        
        # Method to set control mode
        def set_control_mode(self, control_mode):
            control_modes = {"PID": 1, "SPEED": 0}

            if control_mode in control_modes:
                # Send data
                putRegister(self.REG_RECEIVE_CONTROL_MODE, control_modes.get(control_mode))
                print(f"Control mode: {control_mode}")
            else:
                print("Incorrect mode.")

        # Method to set individual PID tuning
        def set_pid_tuning(self, motor, tuning, value):
            tuning_registers = {"A": {"KP": self.REG_RECEIVE_KP_A, "KI": self.REG_RECEIVE_KI_A, "KD": self.REG_RECEIVE_KD_A},
                                "B": {"KP": self.REG_RECEIVE_KP_B, "KI": self.REG_RECEIVE_KP_B, "KD": self.REG_RECEIVE_KD_B}}
            
            if motor in tuning_registers:
                if tuning in tuning_registers.get(motor):
                    # Send data
                    putRegister(tuning_registers.get(motor).get(tuning), value)
                    print(f"PID tuning {tuning} for motor {motor} set to: {value}")
                else:
                    print("Incorrect tuning.")
            else:
                print("Incorrect motor.")

         # Method to set PID tunings
        def set_pid_tunings(self, motor, kp, ki, kd):
            self.set_pid_tuning(motor, "KP", kp)
            self.set_pid_tuning(motor, "KI", ki)
            self.set_pid_tuning(motor, "KD", kd)

        # Method to set PID setpoint
        def set_pid_setpoint(self, motor, value):
            setpoint_registers = {"A": [self.REG_RECEIVE_SETPOINT_A, self.REG_RECEIVE_SETPOINT_A_DEC],
                                  "B": [self.REG_RECEIVE_SETPOINT_B, self.REG_RECEIVE_SETPOINT_B_DEC]}
            
            if motor in setpoint_registers:
                r1, r2 = setpoint_registers.get(motor);
                self.rprk.send_fractional_number(value, r1, r2)
                print(f"PID setpoint for motor {motor} set to: {value}")
            else:
                print("Incorrect motor.")

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
                print("Incorrect sensor name.")
                                
    class Joystick:
        def __init__(self):
            self.REG_SEND_MSG_JOYSTICK = 60 # Joystick send messages
            self.REG_RECEIVE_MSG_JOYSTICK = 61 # Joystic receive messages
            self.REG_SEND_DATA_JOYSTICK = 62 # Joystick send data (direction)
        
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
                print(f"Unusual value read from the register: {direction_data}")
            
    class InfraredSensor:
        def __init__(self):
            # I2C MUX communication 
            self.REG_SEND_IR = 10 # Serial register to send the IR data to
        
            # Get data from the register
        def get_infrared_distance(self):
            return getRegister(self.REG_SEND_IR)
