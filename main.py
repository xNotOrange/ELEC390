import time
import threading
from picarx import Picarx
from pid import PIDController
from pathfinding import findpath, find_nearest_node
from map import makegraph
import readchar
import os

# Constants
POWER = 50  # Default speed of vehicle
forward = 1
backward = 2

# Colours
green = 1
red = 2
yellow = 3
colour = green  # initial state of light

# States
Obstacle = 1
TrafficLight = 2
StopSign = 3

# PID Controller
pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)

# Initialize vehicle
px = Picarx(ultrasonic_pins=['D2', 'D3'], grayscale_pins=['A0', 'A1', 'A2'])

# Create the graph from the SVG map
graph = makegraph('map.svg')

# Starting and end positions
start_pos = (2, 2)  # Start point (in meters)
end_pos = (7, 7)    # End point (in meters)

# Global state variable
state = None
obstacle_detected = False
path = []

# Function to turn left by an angle
def turnleft(angle, speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)

# Function to turn right by an angle
def turnright(angle, speed):
    px.set_dir_servo_angle(angle)
    px.forward(speed)

# Function to move straight
def movestraight(speed):
    px.set_dir_servo_angle(0)
    px.forward(speed)

# Function to stop the vehicle
def stopcar():
    px.stop()
    px.set_dir_servo_angle(0)

# Function for deceleration
def deccelerate(direction, speed, target):
    if direction == forward:
        while speed > target:
            movestraight(speed)
            time.sleep(2)
            speed -= 5
    elif direction == backward:
        while speed > target:
            px.backward(speed)
            time.sleep(2)
            speed -= 5

# Function for acceleration
def accelerate(direction, speed, target):
    if direction == forward:
        while speed < target:
            movestraight(speed)
            time.sleep(2)
            speed += 5
    elif direction == backward:
        while speed < target:
            px.backward(speed)
            time.sleep(2)
            speed += 5

# Function to follow lines with the grayscale sensor
def followLine():
    while True:
        greyscalevalue = px.get_grayscale_data()
        greyscalestate = px.get_line_status(greyscalevalue)
        
        if greyscalestate == [0, 0, 0]:
            stopcar()  # Stop if all sensors are on the line
        elif greyscalestate == [1, 1, 1]:
            movestraight(POWER)  # Move straight if all sensors are on the background
        elif greyscalestate[0] == 0:
            turnright(20, POWER)  # Turn right if left sensor is on the line
        elif greyscalestate[1] == 0:
            movestraight(POWER)  # Continue straight if the middle sensor is on the line
        elif greyscalestate[2] == 0:
            turnleft(20, POWER)  # Turn left if right sensor is on the line

# Function to detect obstacles with the ultrasonic sensor
def detectObstacles():
    global obstacle_detected
    safeDistance = 100
    global dangerDistance
    dangerDistance = 40
    
    while True:
        distance = round(px.ultrasonic.read(), 2)
        if distance >= safeDistance:
            obstacle_detected = False
        elif distance >= dangerDistance:
            obstacle_detected = True
            state = Obstacle
            return

def avoidObstacle():
    global state, path, obstacle_detected

    #Stop the vehicle
    stopcar()

    wait_time = 0 

    while wait_time < 10:
        #check again for obstacle
        distance = round(px.ultrasonic.read(), 2)
        
        if distance == dangerDistance:
            follow_path()
            return

        #Wait for 2 seconds before checking again
        time.sleep(2)
        wait_time += 2

    #Re-plan the path from the current position
    start_pos = (px.get_position_x(), px.get_position_y())
    start_node = find_nearest_node(graph, start_pos)
    end_node = find_nearest_node(graph, end_pos)
    
    #find new path
    path = findpath(graph, start_node, end_node)

    #follow path if there is one
    if path:
        follow_path()
    else:
        stopcar()

# Pathfinding
def plan_path():
    global path
    start_node = find_nearest_node(graph, start_pos)
    end_node = find_nearest_node(graph, end_pos)
    path = findpath(graph, start_node, end_node)

# Function to follow the planned path
def follow_path():
    global path, obstacle_detected
 
    for node in path:
        if obstacle_detected:
            avoidObstacle()
            return
        node_pos = graph.nodes[node]['pos']
        move_towards(node_pos)

def move_towards(position):
    current_position = (px.get_position_x(), px.get_position_y())
    target_x, target_y = position
    error_x = target_x - current_position[0]
    error_y = target_y - current_position[1]
    error_distance = (error_x**2 + error_y**2)**0.5
    
    # PID control for smooth movement
    current_time = time.time()
    dt = current_time - getattr(move_towards, 'last_time', current_time)
    move_towards.last_time = current_time
    correction = pid.update(error_distance, dt)
    
    if correction > 0:
        movestraight(POWER)
    elif correction < 0:
        px.backward(POWER)

    if error_distance < 0.1:  # Close enough to the target
        stopcar()

# Function to determine traffic light color (camera placeholder)
def determinecolour():
    global colour
    # Here you should implement the camera logic to detect the traffic light color
    colour = green  # Placeholder
    return colour

def redlight():
    deccelerate(forward, 50, 5)
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()
    while determinecolour() == red or determinecolour() == yellow:
        stopcar()
    accelerate(forward, 50, 50)

def yellowlight():
    redlight()

def stopsign():
    deccelerate(forward, 50, 5)
    greyscalevalue = px.get_grayscale_data()
    greyscalestate = px.get_line_status(greyscalevalue)
    if greyscalestate == [0, 0, 0]:
        stopcar()
    time.sleep(5)  # Stop for 5 seconds at the stop sign
    accelerate(forward, 0, 50)
    movestraight(50)

# Main function
def main():
    global state
    threading.Thread(target=followLine, daemon=True).start()
    threading.Thread(target=detectObstacles, daemon=True).start()
    threading.Thread(target=plan_path, daemon=True).start()
    
    try:
        while True:
            if state == Obstacle:
                print("Obstacle detected, avoiding...")
                stopcar()  # Stop and handle obstacle
                avoidobstacle()  # Placeholder for actual obstacle avoidance
            elif state == StopSign:
                print("Stop sign detected, stopping...")
                stopsign()
            elif state == TrafficLight:
                colour = determinecolour()
                print(f"Traffic light colour: {colour}")
                if colour == red:
                    redlight()
                elif colour == green:
                    movestraight(POWER)
                elif colour == yellow:
                    yellowlight()
            time.sleep(0.1)
    finally:
        stopcar()

# Start the main function
if __name__ == "__main__":
    main()
