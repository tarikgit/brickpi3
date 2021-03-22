from __future__ import print_function
from __future__ import division

import time
import brickpi3
import random
import math
import time
import numpy as np

BP = brickpi3.BrickPi3()

motor_r = BP.PORT_A
motor_l = BP.PORT_D


###  Main  ###

BP.set_motor_dps(motor_l,35)
BP.set_motor_dps(motor_r,35)

time.sleep(10)

BP.set_motor_dps(motor_l,0)
BP.set_motor_dps(motor_r,0)
