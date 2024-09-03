#
# ARBPi.py - Arduino Robotics Board
# A Python library to compliment the ARB Arduino library
# when using the ARB platfrom in conjuncion with a Raspberry Pi
# June 2022
#

import ctypes 

# The ctypes library lets us use C shared libraries from within Python, allowing us to keep feature parity
# between the C and Python versions of the ARBPi library, but it requires a little extra code on the
# Python side to properly map the functions and datatypes used in the C library to Python equivalents

# Define constants for the two serial devices
SERIAL = "/dev/ttyUSB0"

_ARBPi = ctypes.CDLL('./libARBPi.so') # Open the libARBPi shared library, assigned to the object _ARBPi

# Define the argument types for each C functions to be used that takes arguments
_ARBPi.ARBPiSetup.argtypes = (ctypes.c_char_p,)
_ARBPi.getRegister.argtypes = (ctypes.c_int,)
_ARBPi.putRegister.argtypes = (ctypes.c_int, ctypes.c_byte)

# Define the return types for each C function
_ARBPi.getRegister.restype = ctypes.c_byte

# Define Python-equivalent functions that put arguments in the correct type and calls the external C function
def ARBPiSetup(serialPath):
    global _ARBPi
    _ARBPi.ARBPiSetup(ctypes.c_char_p(serialPath.encode('ascii')))

def getRegister(reg):
    global _ARBPi
    return int(_ARBPi.getRegister(ctypes.c_int(reg)))

def putRegister(reg, data):
    global _ARBPi
    _ARBPi.putRegister(ctypes.c_int(reg), ctypes.c_byte(data))
