import RPi.GPIO as GPIO
from Adafruit_PCA9685 import PCA9685
import time


class SolarMovement(object):
    def __int__(
        self,
        pulneg=11, dirpos=13,
        dirneg=15, enblpin=12,
        servo_increment=1 
    ):
        # Stepper Motor
        self.__pulneg = pulneg
        self.__dirpos = dirpos
        self.__dirneg = dirneg
        self.__enblpin = enblpin
        self.GPIO = GPIO
        # initialize GPIO to BOARD mode
        self.GPIO.setmode(self.GPIO.BOARD)
        # initialize each pins as output pins
        self.GPIO.setup(self.__pulneg, self.GPIO.OUT)
        self.GPIO.setup(self.__dirpos, self.GPIO.OUT)
        self.GPIO.setup(self.__dirneg, self.GPIO.OUT)
        self.GPIO.setup(self.__enblpin, self.GPIO.OUT)
        # set initial state of each outpins
        self.GPIO.output(self.__pulneg, False)
        self.GPIO.output(self.__dirpos, False)
        self.GPIO.output(self.__dirneg, False)
        self.GPIO.output(self.__enblpin, True)
        self.direction = None
        # Servo Motor
        self.servo = PCA9685(0x40)
        self.servo.set_pwm_freq(60)
        self.servo_min = 125
        self.servo_max = 625
        self.servo_initial = 375
        self.servo_current = self.servo_initial
        self.__servo_increment = int(servo_increment)
        
    def stepper_move_left(self):
        self.GPIO.output(self.__dirpos, False)
        self.GPIO.output(self.__dirneg, True)
        self.direction = False
        self.__stepper_move()

    def stepper_move_right(self):
        self.GPIO.output(self.__dirpos, True)
        self.GPIO.output(self.__dirneg, False)
        self.direction = True
        self.__stepper_move()

    def __stepper_move(self):
        for step in range(0, 134, 1):
            self.GPIO.output(self.__pulneg, True)
            time.sleep(0.00035)
            self.GPIO.output(self.__pulneg, False)
            time.sleep(0.00035)
            
    def stepper_enable(self):
        self.GPIO.output(self.__enblpin, True)
        
    def stepper_disable(self):
        self.GPIO.output(self.__enblpin, False)
        
            
    def servo_right(self):
        self.servo_current = self.servo_current - self.__servo_increment
        self.servo_current = (self.servo_min
                              if self.servo_current <= self.servo_min
                              else self.servo_current)
        self.__servo_move()

    def servo_left(self):
        self.servo_current = self.servo_current + self.__servo_increment
        self.servo_current = (self.servo_max
                              if self.servo_current >= self.servo_max
                              else self.servo_current)
        self.__servo_move()

    def __servo_move(self):
        self.servo.set_pwm(0, 0, self.servo_current)
        time.sleep(0.010)

    def set_servo_increment(self, servo_increment):
        self.__servo_increment = int(servo_increment)

    def get_servo_increment(self):
        return self.__servo_increment

    def clean_up(self):
        self.GPIO.cleanup()
