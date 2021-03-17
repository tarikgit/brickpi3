import brickpi3
BP = brickpi3.BrickPi3()

motor_r = BP.PORT_D
motor_l = BP.PORT_A

N = 100

speed = 10
distance = 10

def stop_motor():
    BP.set_motor_dps(motor_l,0)
    BP.set_motor_dps(motor_r,0)
    
def straight(speed,distance):
    while True:
        BP.set_motor_dps(motor_l,speed)
        BP.set_motor_dps(motor_r,speed)
        #assert_touch(t_sensor)
        #if abs(BP.get_motor_encoder(motor_l)) > abs(distance*deg_per_cm):
        #    reset_motor()
        #    stop_motor()
        #    return
    


try:

    while(True):
        BP.set_motor_power(BP.PORT_C, 50)
        BP.set_motor_power(BP.PORT_B, 50)

    
    
#Main:
    particleSet=[]
    for i in range(N):
        particleSet.append([0,0,0])

    
    while True:
        #Wx = input('Please input a target X: (in range [-3,3])')*100.0
    	#while abs(Wx) > 300:
        #    Wx = input('Please input a target X: ')*100
        #Wy = input('Please input a target Y: (in range [-3,3])')*100.0
    	#while abs(Wy) > 300:
    	#    Wy = input('Please input a target Y:')*100
        #particleSet = go_to(particleSet,Wx,Wy)
        #print((estimate_position(particleSet)))
        
        straight(speed,distance)


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 fi    
