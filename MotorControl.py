from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from time import sleep


# GPIO 6 is used for Generating Software PWM
# GPIO 13 & GPIO 19 are used for Motor control pins as per schematic


class MotorControl:
    
    def __init__(self):
        
        # Declare Pins for pwm and motor
        self.PWM_PIN_MOT1 = 21				# Pwm Pin     GPIO 21
        self.OUT_PIN_MOT1 = 20				# Motor Pin 1 GPIO 16
        self.OUT_PIN_MOT2 = 16				# Motor Pin 2 GPIO 20
        self.OUT_PIN_STD  = 12				# Standby Pin GPIO 12
        
        self.MotorSpeed = 1.0

        # PWMOutputDevice takes  BCM_PIN number
        # Active High 
        # intial value
        # PWM Frequency
        # and Pin_factory which can be ignored
        self.pwm_pin_mot1 = PWMOutputDevice (self.PWM_PIN_MOT1,True, 0, 1200)

        # DigitalOutputDevice take 
        # Pin Nuumber
        # Active High
        #Initial Value
        self.cw_pin_mot1 = DigitalOutputDevice (self.OUT_PIN_MOT1, True, 0)
        self.ccw_pin_mot1 = DigitalOutputDevice (self.OUT_PIN_MOT2, True, 0)
        
        self.pin_STD = DigitalOutputDevice (self.OUT_PIN_STD, True, 1)

    def RotateMotorCW(self):

        self.pwm_pin_mot1.value = self.MotorSpeed
        self.cw_pin_mot1.value = 1
        self.ccw_pin_mot1.value = 0


    def RotateMotorCCW(self):

        self.pwm_pin_mot1.value = self.MotorSpeed
        self.cw_pin_mot1.value = 0
        self.ccw_pin_mot1.value = 1


    def StopMotor(self):
        self.cw_pin_mot1.value = 0
        self.ccw_pin_mot1.value = 0
        self.pwm_pin_mot1.value = 0


    def lock(self):
        self.RotateMotorCCW()
        sleep(4)
        self.StopMotor()

    def unlock(self):
        self.RotateMotorCW()
        sleep(4)
        self.StopMotor()