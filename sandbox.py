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
# Define the two sensors
 t_sensor = BP.PORT_3
 us_sensor = BP.PORT_4
# TBD
 x_axis_bias = 150
 y_axis_bias = -80
# TBD
 sigmaE=0
 sigmaF=0
 sigmaG=0
# TBD    
 x_scale = 5
 y_scale = -5
# TBD
 OFFSET = 1.01
 deg_per_cm = 870/40.0
 deg_per_deg = 292/90.0
# TBD    
 N = 100


def particle_forward(position,distance):
    for i in range(0,N):
        nextX = position[i][0] + (distance + random.gauss(0, sigmaE)) * math.cos(math.radians(position[i][2]))
        nextY=position[i][1] + (distance + random.gauss(0, sigmaE)) * math.sin(math.radians(position[i][2]))
        position[i][0] = nextX
        position[i][1] = nextY
        position[i][2] += random.gauss(0,sigmaF)
    return position

def particle_turn(position,angle):
    for i in range(0,N):
        position[i][2]+=angle+random.gauss(0,sigmaG)
    return position

def plot_particles(position):    
    points = []
    for row in position:
        point=((row[0]+x_axis_bias)*x_scale,(row[1]+y_axis_bias)*y_scale,row[2])
        points.append(point)
    #print("drawParticles:"+str(points))

def initialise_ports():
    reset_motor()
    #BP.set_sensor_type(t_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

def straight(speed,distance):
    while True:
        BP.set_motor_dps(motor_l,speed)
        BP.set_motor_dps(motor_r,speed*OFFSET)
        #assert_touch(t_sensor)
        if abs(BP.get_motor_encoder(motor_l)) > abs(distance*deg_per_cm):
            reset_motor()
            stop_motor()
            return

def turn(turn_speed,angle):
    while True:
    	if angle > 0:
                BP.set_motor_dps(motor_l,turn_speed)    # set motor A's target position to the current position of motor D
                BP.set_motor_dps(motor_r,-turn_speed*OFFSET)
    	if angle <= 0:
    	    BP.set_motor_dps(motor_l,-turn_speed)
    	    BP.set_motor_dps(motor_r,turn_speed*OFFSET)
            #assert_touch(t_sensor)
            if abs(BP.get_motor_encoder(motor_l)) > abs(angle*deg_per_deg):
                reset_motor()
                stop_motor()
                return

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

def reset_motor():
    BP.offset_motor_encoder(motor_l, BP.get_motor_encoder(motor_l))
    BP.offset_motor_encoder(motor_r, BP.get_motor_encoder(motor_r))

def stop_motor():
    BP.set_motor_dps(motor_l,0)
    BP.set_motor_dps(motor_r,0)

def normalise_angle(angle):
    turn_angle = angle
    while turn_angle < -180:
        turn_angle += 360
    while turn_angle > 180:
        turn_angle -= 360
    return turn_angle

def print_line():
   # print("drawLine:" + str((x_axis_bias*x_scale,y_axis_bias*y_scale,(x_axis_bias+40)*x_scale,(y_axis_bias)*y_scale)))
   # print("drawLine:" + str((x_axis_bias*x_scale,y_axis_bias*y_scale,x_axis_bias*x_scale,(y_axis_bias+40)*y_scale)))
    return

def estimate_position(particleSet):
    cur_x = 0
    cur_y = 0
    theta_cur = 0

    for row in particleSet:
        cur_x += row[0]
        cur_y += row[1]
        theta_cur += row[2]

    cur_x = cur_x*1.0/N
    cur_y = cur_y*1.0/N
    theta_cur = theta_cur*1.0/N
    theta_cur = normalise_angle(theta_cur)
    return cur_x,cur_y,theta_cur

def navigate_calc(tar_x,tar_y,cur_x,cur_y,theta_cur):
    print((tar_x-cur_x))
    alpha = arctan_normalise((tar_y-cur_y),(tar_x-cur_x))
    turn_angle = alpha - math.radians(theta_cur)#in radians
    turn_angle = turn_angle * 180 / np.pi
    turn_angle = normalise_angle(turn_angle)
    move_dist = math.sqrt((tar_y-cur_y)**2+(tar_x-cur_x)**2)

    return move_dist,turn_angle

def arctan_normalise(a,b):
    if b>=0:
        return np.arctan(a/b)
    if a>=0:
        return np.arctan(a/b)+np.pi
    else:
        return np.arctan(a/b)-np.pi

def go_to(particleSet,tar_x,tar_y):
    
    c_x,c_y,c_z = estimate_position(particleSet)
    move_dist,turn_angle = navigate_calc(tar_x,tar_y,c_x,c_y,c_z)
    particleSet = particle_turn(particleSet,turn_angle)
    particleSet = particle_forward(particleSet,move_dist)
    plot_particles(particleSet)
    print('estimate: ',(estimate_position(particleSet)))
    print('forward: ', move_dist)
    print('turn: ',turn_angle)
    print('target: ',tar_x,tar_y)
    print_line()
    print(turn_angle)
    turn(30, turn_angle)
    straight(60, move_dist)
    print('estimate: ',(estimate_position(particleSet)))
    
    return particleSet


try:
    try:
        initialise_ports()

    except IOError as error:
        print(error) 
    try:
        target = BP.get_motor_encoder(motor_r) # read motor D's position
    except IOError as error:
        print(error)

#Main:
    particleSet=[]
    for i in range(N):
        particleSet.append([0,0,0])

    
    while True:
    #    Wx = input('Please input a target X: (in range [-3,3])')*100.0
    #	while abs(Wx) > 300:
    #        Wx = input('Please input a target X: ')*100
    #    Wy = input('Please input a target Y: (in range [-3,3])')*100.0
    #	while abs(Wy) > 300:
    #	    Wy = input('Please input a target Y:')*100
    #    particleSet = go_to(particleSet,Wx,Wy)
    #    print((estimate_position(particleSet)))
    
    straight(speed,distance):


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 fi
