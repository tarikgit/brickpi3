from __future__ import print_function 
from __future__ import division

import time     
import brickpi3 
import random
import math
import time
import numpy as np

BP = brickpi3.BrickPi3()
motor_r = BP.PORT_D
motor_l = BP.PORT_A
t_sensor = BP.PORT_3
us_sensor = BP.PORT_4
x_axis_bias = 40
y_axis_bias = -240
x_scale = 3
y_scale = -3
N = 100


deg_per_cm = 870/40.0
deg_per_deg = 310/90.0
OFFSET_s = 0.987
OFFSET_t = 1.0

SENSOR_VAR = 2
sigmaE = 1.0/20.0
sigmaF = 0.15/20.0
sigmaG = 0.4


walls = [[0.0,0.0,0.0,168.0], [0.0,168.0,84.0,168.0], [84.0,126.0,84.0,210.0], [84.0,210.0,168.0,210.0],[168.0,210.0,168.0,84.0],[168.0,84.0,210.0,84.0], [210.0,84.0,210.0,0.0], [210.0,0.0,0.0,0.0]]

def initialise_ports():
    reset_motor()
    #BP.set_sensor_type(t_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(us_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

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

'''
Functions of particles updates
'''

def particle_forward(particleSet,distance):
    for i in range(0,N):
        nextX = particleSet[i][0] + (distance + random.gauss(0, sigmaE*distance)) * math.cos(math.radians(particleSet[i][2]))
        nextY = particleSet[i][1] + (distance + random.gauss(0, sigmaE*distance)) * math.sin(math.radians(particleSet[i][2]))
        particleSet[i][0] = nextX
        particleSet[i][1] = nextY
        particleSet[i][2] += random.gauss(0,sigmaF*distance)
    return particleSet

def particle_turn(particleSet,angle):
    for i in range(0,N):
        particleSet[i][2] += angle+random.gauss(0,sigmaG)
    return particleSet

def weight_update(particleSet):
    z = get_sonar_reading()
    #test
    c_x,c_y,c_theta = estimate_position(particleSet)
    print('if horizontal : ' + str(c_x+z))
    print('if vertical : ' + str(c_y+z))

    weight_sum = 0
    for i in range(0,N):
        weight = calculate_likelihood(particleSet[i][0],particleSet[i][1],particleSet[i][2],z)
        particleSet[i][3] = weight
        weight_sum += weight
    for i in range(0,N):
        particleSet[i][3] = particleSet[i][3]/weight_sum
    #test
    #print('weight sum: ' + str(round(weight_sum,5)))
    return particleSet

def resampling(particleSet):
    cum_sum = 0
    cdf = [];
    newParticles = []
    for i in range(0,N):
        newParticles.append([0,0,0,0.01])
    for i in range(0,N):
        cum_sum += particleSet[i][3]
        cdf.append(cum_sum)
    cdf[-1] = 1.0
    #test
    #print('cdf' + str(cdf))

    for i in range(0,N):
        rand = random.uniform(0,1)
        for j in range(0,N):
            if rand <= cdf[j]:
                newParticles[i][0] = particleSet[j][0]
                newParticles[i][1] = particleSet[j][1]
                newParticles[i][2] = particleSet[j][2]
                #print(str(i) + ' sampled: ' + str(j) + ' , with weight: ' + str(round(particleSet[j][3],5)))
                break

    return newParticles


'''
Functions of plotting
'''

def plot_particles(particleSet):    
    points = []
    for row in particleSet:
        point = to_screen_point(row[0],row[1],row[2],row[3]*10)
        points.append(point)
    #test
    #print(str(points))
    print("drawParticles:"+str(points))

def to_screen_point(x,y,theta,weight):
    return ((x+x_axis_bias)*x_scale,(y+y_axis_bias)*y_scale,theta,weight)

def to_screen_line(x1,y1,x2,y2):
    return ((x1+x_axis_bias)*x_scale,(y1+y_axis_bias)*y_scale,(x2+x_axis_bias)*x_scale,(y2+y_axis_bias)*y_scale)

def print_axis():
    print("drawLine:" + str(to_screen_line(0,0,40,0)))
    print("drawLine:" + str(to_screen_line(0,0,0,40)))

def print_walls():
    for i in range(0,8):
        print("drawLine:" + str(to_screen_line(walls[i][0],walls[i][1],walls[i][2],walls[i][3])))

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

def turn(turn_speed,angle):
    while True:
        if angle > 0:
            BP.set_motor_dps(motor_l,-turn_speed)    # set motor A's target position to the current position of motor D
            BP.set_motor_dps(motor_r,turn_speed)
            if abs(BP.get_motor_encoder(motor_l)) > abs(angle*deg_per_deg):
                reset_motor()
                stop_motor()
                return
        if angle <= 0:
            BP.set_motor_dps(motor_l,turn_speed)
            BP.set_motor_dps(motor_r,-turn_speed)
            #assert_touch(t_sensor)
            if abs(BP.get_motor_encoder(motor_l)) > abs(angle*deg_per_deg*OFFSET_t):
                reset_motor()
                stop_motor()
                return

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
def estimate_position(particleSet):
    cur_x = 0
    cur_y = 0
    theta_cur = 0

    for row in particleSet:
        cur_x += row[0]*row[3]
        cur_y += row[1]*row[3]
        theta_cur += row[2]*row[3]

    theta_cur = normalise_angle(theta_cur)
    return cur_x,cur_y,theta_cur

def navigate_calc(tar_x,tar_y,cur_x,cur_y,theta_cur):

    alpha = arctan_normalise((tar_y-cur_y),(tar_x-cur_x))
    turn_angle = alpha - math.radians(theta_cur)#in radians
    turn_angle = turn_angle * 180 / np.pi
    turn_angle = normalise_angle(turn_angle)
    move_dist = math.sqrt((tar_y-cur_y)**2+(tar_x-cur_x)**2)

    return move_dist,turn_angle

def go_to(particleSet,tar_x,tar_y):
    
    c_x,c_y,c_theta = estimate_position(particleSet)
    move_dist,turn_angle = navigate_calc(tar_x,tar_y,c_x,c_y,c_theta)  
    print('\n\n\nPlanning to move '+ str(move_dist) + 'and turn ' + str(turn_angle) + 'to' + str((tar_x,tar_y)))
    #turn the angle first

    turn(130, turn_angle)
    particleSet = particle_turn(particleSet,turn_angle)
    plot_particles(particleSet)
    time.sleep(0.5)
    particleSet = weight_update(particleSet)
    particleSet = resampling(particleSet)
    plot_particles(particleSet)

    
    #print('estimate after turning: ',(estimate_position(particleSet)))

    # while move_dist > 20.0:
    #     if move_dist <= 35.0:
    #         break
    #     straight(200, 20)
    #     particleSet = particle_forward(particleSet,20)
    #     plot_particles(particleSet)
    #     time.sleep(0.5)
    #     particleSet = weight_update(particleSet)
    #     particleSet = resampling(particleSet)
    #     plot_particles(particleSet)
    #     move_dist = move_dist - 20.0


    straight(200, move_dist)
    particleSet = particle_forward(particleSet,move_dist)
    plot_particles(particleSet)
    time.sleep(0.5)
    particleSet = weight_update(particleSet)
    particleSet = resampling(particleSet)
    plot_particles(particleSet)
    print('estimate after reached new point: ',(estimate_position(particleSet)))
    return particleSet

def calculate_likelihood(x,y,theta,z):
    # find m:
    mlist = []
    #print(str(x),str(y),str(theta),str(z))
    for i in range(0,8):
        Ax = walls[i][0]
        Ay = walls[i][1]
        Bx = walls[i][2]
        By = walls[i][3]
        # change the inequality to smaller than epsilon?
        if ((By-Ay)*math.cos(math.radians(theta))-(Bx-Ax)*math.sin(math.radians(theta))) != 0:
            m = ((By-Ay)*(Ax-x)-(Bx-Ax)*(Ay-y))/((By-Ay)*math.cos(math.radians(theta))-(Bx-Ax)*math.sin(math.radians(theta)))
        else:
            continue
        if (m <= 0
         or (Ay == By and (x+m*math.cos(math.radians(theta))) > max(Ax,Bx))
          or (Ay == By and (x+m*math.cos(math.radians(theta))) < min(Ax,Bx))
           or (Ax == Bx and (y+m*math.sin(math.radians(theta))) > max(Ay,By))
            or (Ax == Bx and (y+m*math.sin(math.radians(theta))) < min(Ay,By))):
            continue
        mlist.append(m)
    m = min(mlist)
    # calculate likelihood:
    return np.exp(-(z-m)**2/(2*SENSOR_VAR)) + 0.00001


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

#Main:
    particleSet=[]
    for i in range(N):
        particleSet.append([84,30,0,0.01])

    print_walls()
    
    particleSet = go_to(particleSet,104,30)
    particleSet = go_to(particleSet,124,30)
    particleSet = go_to(particleSet,144,30)
    particleSet = go_to(particleSet,164,30)
    #2
    particleSet = go_to(particleSet,180,30)
    #3
    particleSet = go_to(particleSet,180,54)

    particleSet = go_to(particleSet,160,54)
    #4
    particleSet = go_to(particleSet,138,54)

    particleSet = go_to(particleSet,138,74)
    particleSet = go_to(particleSet,138,94)
    particleSet = go_to(particleSet,138,114)
    particleSet = go_to(particleSet,138,134)
    particleSet = go_to(particleSet,138,154)

    #5
    particleSet = go_to(particleSet,138,168)

    #6
    particleSet = go_to(particleSet,114,168)

    particleSet = go_to(particleSet,114,148)
    particleSet = go_to(particleSet,114,128)
    particleSet = go_to(particleSet,114,108)

    #7
    particleSet = go_to(particleSet,114,84)

    #8
    particleSet = go_to(particleSet,84,84)

    particleSet = go_to(particleSet,84,64)
    particleSet = go_to(particleSet,84,44)

    #1
    particleSet = go_to(particleSet,84,30)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 fi

