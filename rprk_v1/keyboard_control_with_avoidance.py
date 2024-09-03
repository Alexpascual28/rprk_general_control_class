from ARBPi import *
from RPRK import RPRK

import time
import threading
import cv2

class KeyboardControl:
    def __init__(self):
        self.user_input = '';
        self.keyboard_thread = threading.Thread(target=self.main)

    def start_keyboard(self):
        self.keyboard_thread.start()

    def get_user_input(self):
        return self.user_input
    
    def getch(self):
        import sys, termios, tty

        fd = sys.stdin.fileno()
        orig = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)  # or tty.setraw(fd) if you prefer raw mode's behavior.
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, orig)

    def main(self):
        while True:
            # Read user input
            self.user_input = self.getch()

class ObstacleAvoidControlWASD:
    def __init__(self, rprk):
        self.is_initialised = False

        self.left_wall_threshold = 6
        self.right_wall_threshold = 6
        self.front_wall_threshold = 7

        self.rprk = rprk
        self.rprk.motors.set_control_mode("SPEED");

        self.keyboard = KeyboardControl()

        condition_list = (self.is_user_input_P, self.obstacle_in_front, self.wall_to_the_right, self.wall_to_the_left)
        
        condition_checking_order = (5, 4, 3, 2, 1)

        self.state_machine = {
                    0: ["Start", self.initialise, (lambda: self.is_initialised,), (1, 0)],
                    1: ["Read Input", self.manual_control_wasd, condition_list, condition_checking_order],
                    2: ["Adjust Right", self.adjust_right, (self.is_user_input_P, self.wall_to_the_left), (5, 2, 1)],
                    3: ["Adjust Left", self.adjust_left, (self.is_user_input_P, self.wall_to_the_right), (5, 3, 1)],
                    4: ["Adjust Backwards", self.adjust_backwards, (self.is_user_input_P, self.obstacle_in_front), (5, 4, 1)],
                    5: ["Finalize", self.finalize, (lambda: self.exit_flag, ), (6, 6)],
                    }

        self.state_index = 0
        self.exit_flag = False
        
        self.wasd_control_thread = threading.Thread(target=self.main)

        self.user_input = ''
        self.previous_speed_input = '0'
        self.previous_direction_input = ''

        self.keyboard.start_keyboard()
        self.is_initialised =  True
        
    def start_wasd_control(self):
        self.wasd_control_thread.start()

    def main(self):
        while self.exit_flag == False:
            current_state = self.state_machine[self.state_index]

            name = current_state[0];
            print("Current state:", name);

            action = current_state[1]
            
            action()

            for index, condition in enumerate(current_state[2] + (lambda: True,)):
                if condition() == True:
                    self.state_index = current_state[3][index]
                    break;

            time.sleep(0.2)

    def initialise(self):
        print("Initialising robot...")
        print("Initialised") if self.is_initialised == True else print("Not initialised")
        print("Enter robot direction (WASD) or speed (0-9):")

    def manual_control_wasd(self):
        # Read user input
        # print("Getting user input")
        self.user_input = self.keyboard.get_user_input()

        # print(f"User input: {self.user_input}")

        speedInput = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")

        # If the user input is 0-9
        if self.user_input in speedInput:
            if self.user_input != self.previous_speed_input:
                self.rprk.motors.set_robot_speed_by_level(int(self.user_input))
                self.previous_speed_input = self.user_input

        directionInput = {"W": "forward", "A": "left", "S": "backward", "D": "right",
                        "w": "forward", "a": "left", "s": "backward", "d": "right"}

        # If the user input is WASD
        if self.user_input in directionInput:
            if self.user_input != self.previous_direction_input:
                direction = directionInput.get(self.user_input) # Get associated signal in directionInput
                self.rprk.motors.change_direction(direction)
                self.previous_direction_input = self.user_input

    def adjust_backwards(self):
        self.rprk.motors.change_direction("backward");
        self.rprk.motors.set_robot_speed_by_level(7);

        time.sleep(0.3)

        self.rprk.motors.change_direction("right");

        time.sleep(0.2)

        direction = {"W": "forward", "A": "left", "S": "backward", "D": "right",
                     "w": "forward", "a": "left", "s": "backward", "d": "right"}.get(self.previous_direction_input) # Get associated signal in directionInput

        self.rprk.motors.change_direction(direction)
        self.rprk.motors.set_robot_speed_by_level(int(self.previous_speed_input));

    def adjust_left(self):
        self.rprk.motors.change_direction("left");
        self.rprk.motors.set_robot_speed_by_level(int(self.previous_speed_input));

        time.sleep(0.5)

        self.rprk.motors.change_direction("forward");

        # direction = {"W": "forward", "A": "left", "S": "backward", "D": "right",
        #              "w": "forward", "a": "left", "s": "backward", "d": "right"}.get(self.previous_direction_input) # Get associated signal in directionInput

        # self.rprk.motors.change_direction(direction)
        # self.rprk.motors.set_robot_speed_by_level(int(self.previous_speed_input));


    def adjust_right(self):
        self.rprk.motors.change_direction("right");
        self.rprk.motors.set_robot_speed_by_level(int(self.previous_speed_input));

        time.sleep(0.5)

        self.rprk.motors.change_direction("forward");

        # direction = {"W": "forward", "A": "left", "S": "backward", "D": "right",
        #              "w": "forward", "a": "left", "s": "backward", "d": "right"}.get(self.previous_direction_input) # Get associated signal in directionInput

        # self.rprk.motors.change_direction(direction)
        # self.rprk.motors.set_robot_speed_by_level(int(self.previous_speed_input));
    
    def is_user_input_P(self):
        if self.user_input == "p" or self.user_input == "P":
            return True
        else:
            return False

    def is_user_input_L(self):
        if self.user_input == "l" or self.user_input == "L":
            return True
        else:
            return False

    def obstacle_in_front(self):
        ir_distance = self.rprk.infrared.get_infrared_distance()
        return ir_distance < self.front_wall_threshold

    def wall_to_the_right(self):
        right_distance = self.rprk.ultrasound.get_ultrasound_distance("right")
        return right_distance < self.right_wall_threshold

    def wall_to_the_left(self):
        left_distance = self.rprk.ultrasound.get_ultrasound_distance("left")
        return left_distance < self.left_wall_threshold

    def finalize(self):
        self.exit_flag = True
        print("Map explored. Finalizing program...")

class RobotVision:
    def __init__(self, rprk):
        self.rprk = rprk
        self.rprk.camera.start_image_acquisition(show_feed=False)

        self.robot_vision_thread = threading.Thread(target=self.main)

    def start_robot_vision(self):
        self.robot_vision_thread.start()

    def main(self):
        while(True):
            # print("Thread 1")
            source = self.rprk.camera.get_current_image()

            if source is not None:
                image = source.copy()
                frame_aruco, corners, ids = self.rprk.camera.detect_aruco(image, show_frame=False)

                for colour in ["red", "blue", "green", "lilac"]:
                    mask, masked_image = self.rprk.camera.detect_colour(image, colour, show_frame=False, frame_name=colour)

                    if mask is not None:
                        for shape_name in ["triangle", "rectangle", "star", "circle"]:
                            shapes = self.rprk.camera.detect_shapes(mask, shape_name)

                            for shape in shapes:
                                cv2.putText(image, f"{colour} {shape[0]}", (shape[2],shape[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
                                cv2.drawContours(image, shape[1], 0, (0,255,0), 3)
                                
                # Draw aruco locations
                result = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

                # print("Showing Image")
                self.rprk.camera.show_image("Current Image", result)

            else:
                # print("Not Showing Image")
                pass

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        rprk = RPRK()
        wasd_control = ObstacleAvoidControlWASD(rprk);
        robot_vision = RobotVision(rprk);

        robot_vision.start_robot_vision()
        wasd_control.start_wasd_control()
        
    except KeyboardInterrupt:
        rprk.motors.set_robot_speed_by_level(0)
        print('Interrupted!')