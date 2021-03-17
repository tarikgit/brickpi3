from __future__ import print_function 
from __future__ import division

import time     
import brickpi3 
import random
import math
import time
import numpy as np

BP = brickpi3.BrickPi3()

# Define the right motor as connected to port MA and
# the left motor as connected to port MD
motor_r = BP.PORT_A      
motor_l = BP.PORT_D

def straight(speed,distance):
    while True:
        BP.set_motor_dps(motor_l,speed)
        BP.set_motor_dps(motor_r,speed)
        
try:
    try:
        initialise_ports()

    except IOError as error:
        print(error) 


        
#Main:
    particleSet=[]
    for i in range(N):
        particleSet.append([0,0,0])

    
    while True:
        speed = input('Please input speed: ')
        distance = input('Please input distance: ')
        straight(speed,distance)


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 fi
