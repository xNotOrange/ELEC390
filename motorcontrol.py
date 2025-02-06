import time
import RPi.GPIO as GPIO

# Steering Motor Pins
steering_pwm = 22 # Physical Pin 15

#Throttle Motors Pins
throttle_pwm = 25 # Physical Pin 22

GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
GPIO.setup(throttle_enable, GPIO.out)
GPIO.setup(steering_enable, GPIO.out)

# Steering Motor Control
steering = GPIO.PWM(steering_enable, 1000) # set the switching frequency to 1000 Hz
steering.stop()

# Throttle Motors Control
throttle = GPIO.PWM(throttle_enable, 1000) # set the switching frequency to 1000 Hz
throttle.stop()

time.sleep(1)

throttle.start(25) # starts the motor at 25% PWM signal-> (0.25 * battery Voltage)
steering.start(100) # starts the motor at 100% PWM signal-> (1.00 * Battery Voltage)

time.sleep(3)

throttle.stop()
steering.stop()