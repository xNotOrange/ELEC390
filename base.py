from picarx import Picarx
import time
from time import sleep 

POWER = 50
SafeDistance = 40   # > 40 safe
DangerDistance = 20 # > 20 && < 40 turn around,
                    # < 20 backward
px = Picarx()
px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo
px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])

# Please run ./calibration/grayscale_calibration.py to Auto calibrate grayscale values
# or manual modify reference value by follow code
# px.set_line_reference([1400, 1400, 1400])

#Function to turn left by an angle
#Angle will need to be positive
def turnleft(angle,speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)

#Function to turn right by an angle
#Angle will need to be negative
def turnright(angle,speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)  

#Function to straighten vehicle then move forward
def movestraight(speed):
    px.set_dir_servo_angle(0)
    px.forward(speed) 
    
#Function to straighten vehicle then move backwards
def reversestraight(speed):
    px.set_dir_servo_angle(0)
    px.backward(speed) 

#Function to stop vehicle and have it straighten the wheels    
def stopcar():
    px.stop()
    px.set_dir_servo_angle(0)

#Function for decceleration - numbers will have to be tweaked based on how smooth it is
def deccelerate(direction, speed, target):
    if direction == forward:
        while speed > target:
            movestraight(speed)
            sleep (2)
            speed = speed - 5
    elif direction == backward:
        while speed > target:
            reversestraight(speed)
            sleep (2)
            speed = speed - 5
            
#Function for acceleration - numbers will have to be tweaked based on how smooth it is
def accelerate(direction, speed, target):
    if direction == forward:
        while speed < target:
            movestraight(speed)
            sleep (2)
            speed = speed + 5
    elif direction == backward:
        while speed < target:
            reversestraight(speed)
            sleep (2)
            speed = speed + 5 
    
#Function for vehicle to follow lines with the grayscale sensor
def followLine():
    try:
        while True:
            greyscalevalue = px.get_grayscale_data()
            greyscalestate = px.get_line_status(greyscalevalue)
            #[leftsensor (0) ,middlesensor (1) ,rightsensor (2)]
            #1 means background, 0 means line
            
            #all sensors are on a line, the vehicle should stop
            if greyscalestate == [0, 0, 0]:
                px.stop()
            #all sensors are on bg, the vehicle can proceed
            elif greyscalestate == [1, 1, 1]:
                movestraight()
            #the left sensor is on a line (the robot needs to move right by an angle to be in the line)
            elif greyscalestate[0] == 0:
                turnright(20)
            #the middle sensor is on a line (this is probably good, the robot can move forward)
            elif greyscalestate[1] == 0:
                movestraight()
            #the right sensor is on a line (the robot needs to move left by an angle to be in the line)
            elif greyscalestate[2] == 0:
                turnleft(20)
            break
    finally:
        sleep(0.000000000000001)

def detectObstacles():
    try:
        while True:
                
def camera():
    try:
        while True:
            #idk how to code this yet
            #basically the camera will do the visual processing stuff, then change the state for main() based on what it sees
        #cameradata = (however the data is getting transmitted)
        #if cameradata ==
            #state == TrafficLight
                #Add in code to determine light colour
                #colour == 
                    #if colour == 
                        #state == Red
        #elif cameradata ==
            #state == 
    finally: 
        
def redlight():
    deccelerate(forward,50, 5)
    #make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0,0,0]:
        stopcar()
    camera()
    while colour = red | yellow
        stopcar()
    state == Green
    
def yellowlight():
    redlight():
        
def stopsign():
    deccelerate(forward,50)
    #make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0,0,0]:
        stopcar()
    time.sleep(5)
    #We may need to write an if case for if the car needs to turn left/right at an intersection but this is fine for now
    accelerate(forward, 0 , 50)
     movestraight(50)
          
def main():
    global state
    try:
        while True:
            
            #LineFollowing code
            followLine():
            if state == Obstacle
            
            #Vehicle sees a stopsign
            elif state == StopSign
                stopsign():
            #Vehicle sees a traffic light
            elif state == TrafficLight
                #Execute based on light colour
                if state == Red
                    redlight():
                elif state == Green
                    accelerate(forward, 0, 50)
                elif state == Yellow
                    yellowlight():

            elif state == 