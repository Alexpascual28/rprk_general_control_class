from ARBPi import *
from RPRK import RPRK

import math

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        rprk = RPRK()

        rprk.motors.advance_robot(10)

    except KeyboardInterrupt:
        print('Interrupted!')