from ARBPi import *
from RPRK import RPRK

import math

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        rprk = RPRK()

        rprk.motors.set_pid_setpoint("A", 100)

    except KeyboardInterrupt:
        print('Interrupted!')