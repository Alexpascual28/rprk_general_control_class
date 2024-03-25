from ARBPi import *
from RPRK import RPRK

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        kpA = 127
        kiA = 127
        kdA = 127

        kpB = 1
        kiB = 1
        kdB = 1

        rprk = RPRK()
        rprk.motors.set_control_mode("PID");
    
        rprk.motors.set_pid_tunings("A", kpA, kiA, kdA)
        # rprk.motors.set_pid_tunings("B", kpB, kiB, kdB)

        rprk.motors.set_pid_setpoint("A", 10)

        while(True):
            speed = rprk.motors.get_current_speed("A")
            print(f"Motor A speed: {speed} CM/S")

            error = rprk.motors.get_current_distance("A")
            print(f"Error: {error} CM")

    except KeyboardInterrupt:
        print('Interrupted!')