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
def turnleft(angle):
    px.set_dir_servo_angle(angle)
    px.forward(POWER)

#Function to turn right by an angle
#Angle will need to be negative
def turnright(angle):
    px.set_dir_servo_angle(angle)
    px.forward(POWER)  

#Function to straighten vehicle then move forward
def movestraight():
    px.set_dir_servo_angle(0)
    px.forward(POWER) 
    
#Function to straighten vehicle then move backwards
def reversestraight():
    px.set_dir_servo_angle(0)
    px.backward(POWER) 

#Function to stop vehicle and have it straighten the wheels    
def park():
    px.stop()
    px.set_dir_servo_angle(0)

#this is example code from the sunfounder website for using the ultrasonic sensor
def ultrasonicsensor():  
    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            print("distance: ",distance)
            if distance >= SafeDistance:
                movestraight()
            elif distance >= DangerDistance:
                turnleft()
                time.sleep(0.1)
            else:
                px.set_dir_servo_angle(-30)
                px.backward(POWER)
                time.sleep(0.5)

    finally:
        px.forward(0)

#this is example code from the sunfounder website for using the grayscale sensor 
def grayscalesensor():
    current_state = None
    px_power = 10
    offset = 20
    last_state = "stop"

    def outHandle():
        global last_state, current_state
        if last_state == 'left':
            px.set_dir_servo_angle(-30)
            px.backward(10)
        elif last_state == 'right':
            px.set_dir_servo_angle(30)
            px.backward(10)
        while True:
            gm_val_list = px.get_grayscale_data()
            gm_state = get_status(gm_val_list)
            print("outHandle gm_val_list: %s, %s"%(gm_val_list, gm_state))
            currentSta = gm_state
            if currentSta != last_state:
                break
        sleep(0.001)

    def get_status(val_list):
        _state = px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
        if _state == [0, 0, 0]:
            return 'stop'
        elif _state[1] == 1:
            return 'forward'
        elif _state[0] == 1:
            return 'right'
        elif _state[2] == 1:
            return 'left'

    if __name__=='__main__':
        try:
            while True:
                gm_val_list = px.get_grayscale_data()
                gm_state = get_status(gm_val_list)
                print("gm_val_list: %s, %s"%(gm_val_list, gm_state))

                if gm_state != "stop":
                    last_state = gm_state

                if gm_state == 'forward':
                    px.set_dir_servo_angle(0)
                    px.forward(px_power)
                elif gm_state == 'left':
                    px.set_dir_servo_angle(offset)
                    px.forward(px_power)
                elif gm_state == 'right':
                    px.set_dir_servo_angle(-offset)
                    px.forward(px_power)
                else:
                        outHandle()
        finally:
            sleep(5)