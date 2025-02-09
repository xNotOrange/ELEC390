from picarx import Picarx
import time
from time import sleep

POWER = 50
SafeDistance = 40  # > 40 safe
DangerDistance = 20  # > 20 && < 40 turn around, < 20 backward

px = Picarx(ultrasonic_pins=['D2', 'D3'])  # ultrasonic sensor
px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])  # grayscale sensor

# Function to turn left by an angle
def turnleft(angle, speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)

# Function to turn right by an angle
def turnright(angle, speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)

# Function to straighten the vehicle then move forward
def movestraight(speed):
    px.set_dir_servo_angle(0)
    px.forward(speed)

# Function to straighten the vehicle then move backward
def reversestraight(speed):
    px.set_dir_servo_angle(0)
    px.backward(speed)

# Function to stop the vehicle and have it straighten the wheels
def stopcar():
    px.stop()
    px.set_dir_servo_angle(0)

# Function for deceleration - numbers will have to be tweaked based on how smooth it is
def deccelerate(direction, speed, target):
    if direction == 'forward':
        while speed > target:
            movestraight(speed)
            sleep(2)
            speed = speed - 5
    elif direction == 'backward':
        while speed > target:
            reversestraight(speed)
            sleep(2)
            speed = speed - 5

# Function for acceleration - numbers will have to be tweaked based on how smooth it is
def accelerate(direction, speed, target):
    if direction == 'forward':
        while speed < target:
            movestraight(speed)
            sleep(2)
            speed = speed + 5
    elif direction == 'backward':
        while speed < target:
            reversestraight(speed)
            sleep(2)
            speed = speed + 5

# Function to follow lines with the grayscale sensor
def followLine():
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)

    # all sensors are on a line, the vehicle should stop
    if greyscalestate == [0, 0, 0]:
        stopcar()

    # all sensors are on background, the vehicle can proceed
    elif greyscalestate == [1, 1, 1]:
        movestraight(POWER)

    # the left sensor is on a line (the robot needs to move right by an angle)
    elif greyscalestate[0] == 0:
        turnright(20, POWER)

    # the middle sensor is on a line (this is probably good, the robot can move forward)
    elif greyscalestate[1] == 0:
        movestraight(POWER)

    # the right sensor is on a line (the robot needs to move left by an angle)
    elif greyscalestate[2] == 0:
        turnleft(20, POWER)

#Function to use camera
def camera():
    try:
        while True:
            #idk how to code this yet
            #basically the camera will do the visual processing stuff, then change the state for main() based on what it sees
        #cameradata = (however the data is getting transmitted)
        #if cameradata ==
            #state == TrafficLight

#Function to determine traffic light colours 
def determinecolour():
    
    
def redlight():
    deccelerate('forward', 50, 5)
    # Make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()

    # Wait until the light is no longer red or yellow
    while camera() == 'red' or camera() == 'yellow':
        stopcar()
    accelerate('forward', 50, 50)

def yellowlight():
    redlight()

def stopsign():
    deccelerate('forward', 50, 5)
    # Make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()
    time.sleep(5)  # Stop for 5 seconds
    accelerate('forward', 0, 50)
    movestraight(50)

def main():
    global state
    state = None

    try:
        while True:
            # Line following code
            followLine()

            if state == 'Obstacle':
                detectObstacles()

            # Vehicle sees a stop sign
            elif state == 'StopSign':
                stopsign()

            # Vehicle sees a traffic light
            elif state == 'TrafficLight':
                determinecolour()
                if colour == 'red':
                    redlight()
                elif colour == 'green':
                    accelerate('forward', 0, 50)
                elif colour == 'yellow':
                    yellowlight()
    finally:
        stopcar()

# Start the main function
main()
