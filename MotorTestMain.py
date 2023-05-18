import RPi.GPIO as GPIO
from MotorControl import *
Motor = MotorControl()

#Dcclare the interrupt buttons to lock and unlock
LOCKBUTTON_GPIO = 7
UNLOCKBUTTON_GPIO = 8

#Setup the pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(LOCKBUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(UNLOCKBUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)


#Define callback functions
def lock_button_pressed_callback(channel):
    global IsLocked
    if IsLocked == False:
        print("Motor is Starting to lock the door")
        Motor.lock()
        IsLocked = True
    
def unlock_button_pressed_callback(channel):
    global IsLocked
    if IsLocked == True:
        print("Motor is Starting to unlock the door")
        Motor.unlock()
        IsLocked = False

#Add interrupt callbacks
GPIO.add_event_detect(LOCKBUTTON_GPIO, GPIO.FALLING, 
            callback=lock_button_pressed_callback, bouncetime=100)

GPIO.add_event_detect(UNLOCKBUTTON_GPIO, GPIO.FALLING, 
            callback=unlock_button_pressed_callback, bouncetime=100)

#Declare variable to save the Lock state
IsLocked = False