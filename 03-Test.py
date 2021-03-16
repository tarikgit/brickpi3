from __future__ import print_function 
from __future__ import division

import time     
import brickpi3 
import random
import math
import time
import numpy as np
import csv

BP = brickpi3.BrickPi3()
motor_r = BP.PORT_B
motor_l = BP.PORT_C
motor_s = BP.PORT_D
tl_sensor = BP.PORT_3
tr_sensor = BP.PORT_4
us_sensor = BP.PORT_2
x_axis_bias = 40
y_axis_bias = -240
x_scale = 3
y_scale = -3
N = 80


deg_per_cm = 870/40.0
deg_per_deg = 322/90.0
OFFSET_s = 1.0
OFFSET_t = 1.0
SONAR_RIGHT_ANGLE = 20
SONAR_LEFT_ANGLE = 90
SONAR_SWEEPING_SPEED = 10
# SONAR_MOTOR_TURN_OFFSET
# FAR_SPEED
# NEAR_SPEED
TILT_ANGLE = 42
STRAIGHT_SPEED = 100
TURN_SPEED = 80

SENSOR_VAR = 2
sigmaE = 1.5/20.0
sigmaF = 0.15/20.0
sigmaG = 0.5
BOTTLE_THRESHOLD = 40
TARGET_THRESHOLD = 10
BACKUP_DISTANCE = 12
WALL_BOTTLE_DIFF = 7
HORIZONTAL_PATHWAY = 55
A_TO_B_OFFSET = 3


#walls = [[0.0,0.0,0.0,168.0], [0.0,168.0,84.0,168.0], [84.0,126.0,84.0,210.0], [84.0,210.0,168.0,210.0],[168.0,210.0,168.0,84.0],[168.0,84.0,210.0,84.0], [210.0,84.0,210.0,0.0], [210.0,0.0,0.0,0.0]]

def initialise_ports():
    reset_motor()
    BP.set_sensor_type(tl_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    BP.set_sensor_type(tr_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    BP.set_sensor_type(tl_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(tr_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    #BP.set_motor_limit(motor_r, 0.5, 400)
    #BP.set_motor_limit(motor_l, 0.5, 400)

def reset_motor():
    BP.offset_motor_encoder(motor_l, BP.get_motor_encoder(motor_l))
    BP.offset_motor_encoder(motor_r, BP.get_motor_encoder(motor_r))

def stop_motor():
    BP.set_motor_dps(motor_l,0)
    BP.set_motor_dps(motor_r,0)

def get_sonar_reading():
    reading_sum = 0
    count = 0
    i = 0
    while count < 200 and i < 700:
        i += 1
        try:
            reading = BP.get_sensor(us_sensor)
        except:
            pass
        if reading < 230.0:
            if reading < 20.0:
                reading -= 3.0
            reading_sum += reading
            count += 1
    if count > 0:
        print('reading distance: ' + str(reading_sum/count))
        return reading_sum/count    
    else:
        return 0

def get_sonar_reading_median(max_distance=150):
    reading_list = []
    while len(reading_list) <= 100:
        reading = get_sensor(us_sensor)
        if reading < max_distance:
            reading_list.append(reading)

    return np.median(reading_list)

'''
Functions to control motion of robot
'''
def straight(speed,distance):
    while True:
        BP.set_motor_dps(motor_l,speed)
        BP.set_motor_dps(motor_r,speed*OFFSET_s)
        #assert_touch(t_sensor)
        if abs(BP.get_motor_encoder(motor_l)) > abs(distance*deg_per_cm):
            reset_motor()
            stop_motor()
            return

def backward(speed,distance):
    while True:
        BP.set_motor_dps(motor_l,-speed*OFFSET_s)
        BP.set_motor_dps(motor_r,-speed)
        #assert_touch(t_sensor)
        if abs(BP.get_motor_encoder(motor_l)) > abs(distance*deg_per_cm):
            reset_motor()
            stop_motor()
            return

def turn(turn_speed,angle):
    while True:
        if angle > 0:
            BP.set_motor_dps(motor_l,-turn_speed*OFFSET_s)    # set motor A's target position to the current position of motor D
            BP.set_motor_dps(motor_r,turn_speed)
            if abs(BP.get_motor_encoder(motor_l)) > abs(angle*deg_per_deg):
                reset_motor()
                stop_motor()
                return
        if angle <= 0:
            BP.set_motor_dps(motor_l,turn_speed*OFFSET_s)
            BP.set_motor_dps(motor_r,-turn_speed)
            #assert_touch(t_sensor)
            if abs(BP.get_motor_encoder(motor_l)) > abs(angle*deg_per_deg*OFFSET_t):
                reset_motor()
                stop_motor()
                return
            
def sonar_sweep(turn_speed, angle_right, angle_left):

    while True:
        BP.set_motor_dps(motor_s, turn_speed)    
        motor_encoder = BP.get_motor_encoder(motor_s)
        if abs(motor_encoder) > abs(angle_right):
            BP.set_motor_dps(motor_s,0)
            break

    distances = []
    last_deg = 0
    current_deg_array = []
    while True:
        motor_encoder = BP.get_motor_encoder(motor_s)
        if motor_encoder != last_deg:
            distances.append([-motor_encoder, np.median(current_deg_array)])
            current_deg_array = []
            if abs(motor_encoder) > abs(angle_left):
                BP.set_motor_dps(motor_s,0)
                break
        BP.set_motor_dps(motor_s, -turn_speed)    
        distance_reading = BP.get_sensor(us_sensor)
        current_deg_array.append(distance_reading)
        last_deg = motor_encoder
    print(distances)
    BP.set_motor_position(motor_s, 0)
    return distances
'''
Touch Sensors
'''
def assert_touch(sensor):
    switch = BP.get_sensor(sensor)
    if switch == 1:
        BP.set_motor_dps(motor_l,0)
        BP.set_motor_dps(motor_r,0)
        time.sleep(0.3)
    wait_resume(t_sensor)
    return

def wait_resume(sensor):
    while True:
        time.sleep(0.02)
        if BP.get_sensor(sensor) == 1:
            switchsum = 0
            while BP.get_sensor(sensor) == 1:
                switchsum += 1
                if switchsum > 1000:
                    exit()
            time.sleep(0.3)
            return

'''
Angles
'''

def normalise_angle(angle):
    turn_angle = angle
    while turn_angle < -180:
        turn_angle += 360
    while turn_angle > 180:
        turn_angle -= 360
    return turn_angle

def arctan_normalise(a,b):
    if b>=0:
        return np.arctan(a/b)
    if a>=0:
        return np.arctan(a/b)+np.pi
    else:
        return np.arctan(a/b)-np.pi


'''
Functions for planning
'''

def navigate_calc(tar_x,tar_y,cur_x,cur_y,theta_cur):

    alpha = arctan_normalise((tar_y-cur_y),(tar_x-cur_x))
    turn_angle = alpha - math.radians(theta_cur)#in radians
    turn_angle = turn_angle * 180 / np.pi
    turn_angle = normalise_angle(turn_angle)
    move_dist = math.sqrt((tar_y-cur_y)**2+(tar_x-cur_x)**2)

    return move_dist,turn_angle

def analyse_histogram(histogram):
    last_dist = histogram[0][1]
    mid_index = int(len(histogram)/2)
    index_list = []
    for row in histogram:
        if row[1] == histogram[mid_index][1]:
            index_list.append(row[0])

    return np.median(index_list)




# def update_position_straight(location, dist_traveled, sonar_turn):
#     reset_motor()
#     stop_motor()
#     location[0] += dist_traveled * math.cos(math.radians(location[2]))
#     location[1] += dist_traveled * math.sin(math.radians(location[2]))
#     location[3] += sonar_turn
#     return location

def get_sensor(port):
    try:
        return BP.get_sensor(port)
    except:
        return 255

try:
    try:
        initialise_ports()

    except IOError as error:
        print(error)
    try:
        target = BP.get_motor_encoder(motor_r) # read motor D's position
        BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.NXT_TOUCH)
        time.sleep(1)
        BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        time.sleep(1)

    except IOError as error:
        print(error)

    BP.offset_motor_encoder(motor_s, BP.get_motor_encoder(motor_s))
    current_loc = [84,30,0,0]
    A_end = [195,30]
    B_end = [115,195]
    C_end = [30,153]
    found = [False,False,False]
    BP.set_motor_position(motor_s, -TILT_ANGLE)
    current_loc[3] = TILT_ANGLE
    print(current_loc)
    A_straight_start = BP.get_motor_encoder(motor_l)

    target_count = 0
    bottle_distance_list = []
    on_side = False
    while True:
        BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
        BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
        sonar_reading = get_sensor(us_sensor)

        if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
            BP.set_motor_position(motor_s, 0)
            current_loc[3] = 0
            print('touch!!!!!')
            current_loc[0] += abs(BP.get_motor_encoder(motor_l) - A_straight_start)/deg_per_cm
            reset_motor()
            stop_motor()
            found[0] = True
            print(current_loc)
            break

        if abs(BP.get_motor_encoder(motor_l)) > abs((A_end[0]-current_loc[0])*deg_per_cm):  # if end point is reached

            BP.set_motor_position(motor_s, 0)
            current_loc[3] = 0
            current_loc[0] = A_end[0]
            reset_motor()
            stop_motor()
            print(current_loc)
            break
        if sonar_reading < BOTTLE_THRESHOLD:
            target_count += 1
            bottle_distance_list.append(sonar_reading)
        else:
            target_count = max(0, target_count - 1)

        if target_count > TARGET_THRESHOLD:
            print('found target')
            bottle_distance = np.median(bottle_distance_list)
            current_loc[0] += abs(BP.get_motor_encoder(motor_l) - A_straight_start)/deg_per_cm

            BP.set_motor_position(motor_s, 5)
            current_loc[3] = -5
            reset_motor()
            stop_motor()
            time.sleep(1)

            straight_distance = get_sonar_reading_median(max_distance=256)
            print('straight_distance = '+ str(straight_distance))
            print('bottle_distance = '+ str(bottle_distance))
            if straight_distance - bottle_distance > WALL_BOTTLE_DIFF:
                print('Found the bottle on left hand side!')
                on_side = True            
                turn(TURN_SPEED, TILT_ANGLE)
                current_loc[2] += TILT_ANGLE
                reset_motor()
                stop_motor()

            print(current_loc)
            A_straight_start = BP.get_motor_encoder(motor_l)


            while abs(BP.get_motor_encoder(motor_l)) < abs(bottle_distance+5)*deg_per_cm:
                BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
                BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
                if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
                    print('touch!!!!!')
                    dist_traveled = abs(BP.get_motor_encoder(motor_l) - A_straight_start)/deg_per_cm
                    current_loc[0] += dist_traveled * math.cos(math.radians(current_loc[2]))
                    current_loc[1] += dist_traveled * math.sin(math.radians(current_loc[2]))
                    found[0] = True
                    print(current_loc)
                    break
            break

    print(current_loc)
    reset_motor()
    stop_motor()

    backward(STRAIGHT_SPEED, BACKUP_DISTANCE)
    current_loc[0] -= BACKUP_DISTANCE * math.cos(math.radians(current_loc[2]))
    current_loc[1] -= BACKUP_DISTANCE * math.sin(math.radians(current_loc[2]))

    if on_side: 
        turn(TURN_SPEED, - current_loc[2])
        current_loc[2] = 0

        backward(STRAIGHT_SPEED, current_loc[0]-115)
        current_loc[0] = 115
        
        turn(TURN_SPEED, 90 - A_TO_B_OFFSET)
        current_loc[2] = 90
    else:
        backward(STRAIGHT_SPEED, current_loc[0]-115)
        current_loc[0] = 115

        turn(TURN_SPEED, 90)
        current_loc[2] = 90


    straight(STRAIGHT_SPEED, 79-current_loc[1])
    current_loc[1] = 79

    print(current_loc)



    on_side = False
    time.sleep(0.5)
    BP.set_motor_position(motor_s, TILT_ANGLE)
    current_loc[3] = -TILT_ANGLE
    print(current_loc)

    B_straight_start = BP.get_motor_encoder(motor_l)

    target_count = 0
    bottle_distance_list = []


    while True:
        BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
        BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
        sonar_reading = get_sensor(us_sensor)

        if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
            
            BP.set_motor_position(motor_s, 0)
            current_loc[3] = 0
            print('touch!!!!!')
            current_loc[1] += abs(BP.get_motor_encoder(motor_l) - B_straight_start)/deg_per_cm
            reset_motor()
            stop_motor()
            found[1] = True
            print(current_loc)
            break

        if abs(BP.get_motor_encoder(motor_l)) > abs((B_end[1]-current_loc[1])*deg_per_cm):  # if end point is reached

            BP.set_motor_position(motor_s, 0)
            current_loc[3] = 0
            current_loc[1] = B_end[1]
            print(current_loc)
            reset_motor()
            stop_motor()
            break

        if sonar_reading < BOTTLE_THRESHOLD:
            target_count += 1
            bottle_distance_list.append(sonar_reading)
        else:
            target_count = max(0, target_count - 1)

        if target_count > TARGET_THRESHOLD:
            print('found target')
            bottle_distance = np.median(bottle_distance_list)
            current_loc[1] += abs(BP.get_motor_encoder(motor_l) - B_straight_start)/deg_per_cm

            BP.set_motor_position(motor_s, -5)
            current_loc[3] = 5
            reset_motor()
            stop_motor()
            time.sleep(1)


            straight_distance = get_sonar_reading_median(max_distance=256)
            print('straight_distance = '+ str(straight_distance))
            print('bottle_distance = '+ str(bottle_distance))
            if straight_distance - bottle_distance > WALL_BOTTLE_DIFF:
                print('Found the bottle on right hand side!')            
                turn(TURN_SPEED, -TILT_ANGLE)
                current_loc[2] -= TILT_ANGLE
                on_side = True
                reset_motor()
                stop_motor()
            print(current_loc)

            B_straight_start = BP.get_motor_encoder(motor_l)


            while abs(BP.get_motor_encoder(motor_l)) < abs(bottle_distance+5)*deg_per_cm:
                BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
                BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
                if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
                    print('touch!!!!!')
                    dist_traveled = abs(BP.get_motor_encoder(motor_l) - B_straight_start)/deg_per_cm
                    current_loc[0] += dist_traveled * math.cos(math.radians(current_loc[2]))
                    current_loc[1] += dist_traveled * math.sin(math.radians(current_loc[2]))
                    found[1] = True
                    reset_motor()
                    stop_motor()
                    print(current_loc)
                    break
            break

    print(current_loc)
    reset_motor()
    stop_motor()

    backup = BACKUP_DISTANCE
    if not on_side:
        backup = BACKUP_DISTANCE + 13


    backward(STRAIGHT_SPEED, backup)
    current_loc[0] -= backup * math.cos(math.radians(current_loc[2]))
    current_loc[1] -= backup * math.sin(math.radians(current_loc[2]))

    if on_side:
        turn(TURN_SPEED, 180 - current_loc[2])
        current_loc[2] = 180

        straight(STRAIGHT_SPEED, current_loc[0] - 115)
        current_loc[0] = 115

        # -90 or 270?
        turn(TURN_SPEED, 90)
        current_loc[2] = -90

        
    else:
        turn(TURN_SPEED, 180)
        current_loc[2] = -90

    if current_loc[1] > HORIZONTAL_PATHWAY:
        straight(STRAIGHT_SPEED, current_loc[1] - HORIZONTAL_PATHWAY)
        current_loc[1] = HORIZONTAL_PATHWAY

    turn(TURN_SPEED, -90)
    current_loc[2] = 180

    straight(STRAIGHT_SPEED, current_loc[0] - 30)
    current_loc[0] = 30

    turn(TURN_SPEED, -90)
    current_loc[2] = 90

    print(current_loc)


    on_side = False

    BP.set_motor_position(motor_s, TILT_ANGLE)
    current_loc[3] = -TILT_ANGLE
    print(current_loc)
    C_straight_start = BP.get_motor_encoder(motor_l)

    target_count = 0
    bottle_distance_list = []

    while True:
        BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
        BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
        sonar_reading = get_sensor(us_sensor)

        if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
            BP.set_motor_position(motor_s, 0)
            current_loc[3] = 0
            print('touch!!!!!')
            current_loc[1] += abs(BP.get_motor_encoder(motor_l) - C_straight_start)/deg_per_cm
            found[2] = True
            reset_motor()
            stop_motor()
            print(current_loc)
            break

        if abs(BP.get_motor_encoder(motor_l)) > abs((C_end[1]-current_loc[1])*deg_per_cm):  # if end point is reached
            
            BP.set_motor_position(motor_s, 0)
            current_loc[3] -= TILT_ANGLE
            current_loc[0] = C_end[0]
            print(current_loc)
            reset_motor()
            stop_motor()
            break

        if sonar_reading < BOTTLE_THRESHOLD:
            target_count += 1
            bottle_distance_list.append(sonar_reading)
        else:
            target_count = max(0, target_count - 1)

        if target_count > TARGET_THRESHOLD:
            print('found target')
            bottle_distance = np.median(bottle_distance_list)
            current_loc[1] += abs(BP.get_motor_encoder(motor_l) - C_straight_start)/deg_per_cm

            BP.set_motor_position(motor_s, -5)
            current_loc[3] = 5
            
            time.sleep(1)
            reset_motor()
            stop_motor()

            straight_distance = get_sonar_reading_median(max_distance=256)
            print('straight_distance = '+ str(straight_distance))
            print('bottle_distance = '+ str(bottle_distance))
            if straight_distance - bottle_distance > WALL_BOTTLE_DIFF:
                on_side = True
                print('Found the bottle on right hand side!')            
                turn(TURN_SPEED, -TILT_ANGLE)
                current_loc[2] -= TILT_ANGLE
            reset_motor()
            stop_motor()
            print(current_loc)
            C_straight_start = BP.get_motor_encoder(motor_l)


            while abs(BP.get_motor_encoder(motor_l)) < abs(bottle_distance+5)*deg_per_cm:
                BP.set_motor_dps(motor_l,STRAIGHT_SPEED)
                BP.set_motor_dps(motor_r,STRAIGHT_SPEED*OFFSET_s)
                if BP.get_sensor(tl_sensor) or BP.get_sensor(tr_sensor):
                    print('touch!!!!!')
                    dist_traveled = abs(BP.get_motor_encoder(motor_l) - C_straight_start)/deg_per_cm
                    current_loc[0] += dist_traveled * math.cos(math.radians(current_loc[2]))
                    current_loc[1] += dist_traveled * math.sin(math.radians(current_loc[2]))
                    found[0] = True
                    print(current_loc)
                    break
            break

    print(current_loc)
    reset_motor()
    stop_motor()

    backup = BACKUP_DISTANCE
    if not on_side:
        backup = BACKUP_DISTANCE + 13

    backward(STRAIGHT_SPEED, backup)
    current_loc[0] -= backup * math.cos(math.radians(current_loc[2]))
    current_loc[1] -= backup * math.sin(math.radians(current_loc[2]))

    if on_side:
        turn(TURN_SPEED, -current_loc[2])
        current_loc[2] = 0
    else:
        turn(TURN_SPEED, -90)
        current_loc[2] = 0

    turn(TURN_SPEED, -90)
    current_loc[2] = -90

    if current_loc[1] > HORIZONTAL_PATHWAY:
        straight(STRAIGHT_SPEED, current_loc[1]-HORIZONTAL_PATHWAY)
        current_loc[1] = HORIZONTAL_PATHWAY

    turn(TURN_SPEED, 90)
    current_loc[2] = 0

    straight(STRAIGHT_SPEED, 84 - current_loc[0])
    current_loc[0] = 84
    
    turn(TURN_SPEED, -90)
    current_loc[2] = -90

    time.sleep(0.5)
    current_loc[1] = get_sonar_reading_median()+5
    print('corrected y: ' + str(current_loc[1]))
    time.sleep(0.5)

    BP.set_motor_position(motor_s, 90)

    time.sleep(1)
    current_loc[0] = get_sonar_reading_median()+2
    print('corrected x: ' + str(current_loc[0]))
    

    move_dist, turn_angle = navigate_calc(84,30,current_loc[0],current_loc[1],current_loc[2])
    turn(TURN_SPEED, turn_angle)
    time.sleep(0.5)
    straight(STRAIGHT_SPEED, move_dist)

    


    BP.reset_all()
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 fi
