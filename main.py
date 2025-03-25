from picarx import Picarx
import time
from time import sleep
from pydoc import text
from vilib import Vilib
from time import sleep, time, strftime, localtime
import threading
import readchar
import os
import numpy as np
import heapq
import networkx as nx
from map import makegraph
from pathfinding import findpath, find_nearest_node
import threading
from pid import PIDController

POWER = 50 #Default speed of vehicle
forward = 1
backward = 2

#Colours
green = 1
red = 2
yellow = 3
colour = green #initial state of light

#States
Obstacle = 1
TrafficLight = 2
StopSign = 3

pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)

px = Picarx(ultrasonic_pins=['D2', 'D3'])  # ultrasonic sensor
px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])  # grayscale sensor

# Create the graph from the SVG map
graph = makegraph('map.svg')

#Starting and end positions
start_pos = (2, 2)  # Start point (in meters)
end_pos = (7, 7)    # End point (in meters)

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
    if direction == forward:
        while speed > target:
            movestraight(speed)
            sleep(2)
            speed = speed - 5
    elif direction == backward:
        while speed > target:
            reversestraight(speed)
            sleep(2)
            speed = speed - 5

# Function for acceleration - numbers will have to be tweaked based on how smooth it is
def accelerate(direction, speed, target):
    if direction == forward:
        while speed < target:
            movestraight(speed)
            sleep(2)
            speed = speed + 5
    elif direction == backward:
        while speed < target:
            reversestraight(speed)
            sleep(2)
            speed = speed + 5

# Function to follow lines with the grayscale sensor
def followLine():
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    print("Greyscale Value: ", greyscalevalue)
    print("Greyscale State: ", greyscalestate)
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

#Function to detect obstacles/sudden hazards with the ultrasonic sensor
def detectObstacles():
    #distances so the sensor knows what is 'safe' and what is not
    safeDistance = 100
    dangerDistance = 40
    
    try:
        while True:
            #Read sensor data and save distance, print information to terminal for testing
            distance = round(px.ultrasonic.read(), 2)
            print("distance: ",distance)
            
            #Vehicle proceeds if distance is safe 
            if distance >= safeDistance:
                movestraight(50)
            #Vehicle stops if there is a hazard in the danger zone
            elif distance >= dangerDistance:
                avoidobstacle()
    finally:
        sleep(0.00000000000000001)

#todo: make this better
#the vehicle needs to be able to determine if the obstacle is avoidable or if it is something it needs to stop for
def avoidobstacle():
    stopcar()

#Pathfinding
def findpath():
    # Find the nearest nodes in the graph for the start and end positions
    start_node = find_nearest_node(graph, start_pos)
    end_node = find_nearest_node(graph, end_pos)

    # Find the path using A* algorithm
    path = findpath(graph, start_node, end_node)

# Function to follow the path
def follow_path():
    global path
    if not path:
        print("No path found.")
        return

    for node in path:
        node_pos = graph.nodes[node]['pos']
        #Move towards this node's position
        move_towards(node_pos)

def move_towards(position):

    current_position = (px.get_position_x(), px.get_position_y())
    target_x, target_y = position
    
    #Calculate the error (difference in x and y)
    error_x = target_x - current_position[0]
    error_y = target_y - current_position[1]
    
    error_distance = (error_x**2 + error_y**2)**0.5
    
    #Get the current time to calculate dt for the PID controller
    current_time = time.time()
    dt = current_time - getattr(move_towards, 'last_time', current_time)
    move_towards.last_time = current_time
    
    #Update PID controller with the error distance and dt
    correction = pid.update(error_distance, dt)
    
    #adjust the vehicle's movement
    if correction > 0:
        movestraight(POWER)
    elif correction < 0:
        reversestraight(POWER)

    #stop once the vehicle is close enough to the target
    if error_distance < 0.1:  # Close enough to the target
        stopcar()

#todo: write this   
#Function to use camera
def camera():
    try:
        while True:
            print() #placeholder so this stops popping up as an error
            #idk how to code this yet
            #basically the camera will do the visual processing stuff, then change the state for main() based on what it sees
        #cameradata = (however the data is getting transmitted)
        #if cameradata ==
            #state == TrafficLight
    finally:
        sleep(0.00000000000001)

#Function to determine traffic light colours 
def determinecolour():
    colour = green
    print("Light Colour: ", colour)
    print()#placeholder so this stops popping up as an error
    return colour #placeholder so this stops popping up as an error
    
def redlight():
    deccelerate(forward, 50, 5)
    # Make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()

    # Wait until the light is no longer red or yellow
    while camera() == red or camera() == yellow:
        stopcar()
    accelerate(forward, 50, 50)

def yellowlight():
    redlight()

def stopsign():
    deccelerate(forward, 50, 5)
    # Make sure we stop on the line
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()
    time.sleep(5)  # Stop for 5 seconds
    accelerate(forward, 0, 50)
    movestraight(50)

def main():
    global state
    
    threading.Thread(target=followLine, daemon=True).start()
    threading.Thread(target=detectObstacles, daemon=True).start()
    threading.Thread(target=camera, daemon=True).start()
    threading.Thread(target=follow_path, daemon=True).start()
    
    try:
        while True:
                  
            if state == Obstacle:
                avoidobstacle()
            # Vehicle sees a stop sign
            elif state == StopSign:
                stopsign()
            # Vehicle sees a traffic light
            elif state == TrafficLight:
                determinecolour()
                if colour == red:
                    redlight()
                elif colour == green:
                    movestraight(POWER)
                elif colour == yellow:
                    yellowlight()
    finally:
        stopcar()

# Start the main function
main()
