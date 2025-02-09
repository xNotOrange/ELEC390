#this is example code from the sunfounder website for using the ultrasonic sensor + the movement functions I wrote
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