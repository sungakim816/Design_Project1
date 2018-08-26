from SimpleCV import (
    Camera,
    Color,
    Display,
    DrawingLayer
    )
import RPi.GPIO as GPIO
from Adafruit_PCA9685 import PCA9685
import time


class SolarCamera:
    def __init__(self, window_size=(640, 480), **kwargs):
        while True:  # Initialize the Camera
            try:
                self.cam = Camera()
                self.cam.getImage().flipHorizontal()
            except:
                continue
            else:
                break

        self.image = None
        self.window_size = window_size
        self.display = Display(self.window_size)
        self.__window_center = (self.window_size[0]/2, self.window_size[1]/2)
        self.__distance = None
        self.__blobs = None
        self.__segmented = None
        self.__circles = None
        self.__scope_layer = None
        self.initialize_scope_layer()

    @property
    def is_there_sun(self):
        self.__binarize_image()  # binarize image
        if self.__blobs:
            self.__circles = self.__blobs.filter([b.isCircle(0.2) for b in self.__blobs])
            if self.__circles:
                return True
        return False

    def __binarize_image(self):
        #  dilate the image
        self.__distance = self.image.colorDistance(Color.BLACK).dilate(2)
        #  segment image with colors ranging from 250 to 255
        self.__segmented = self.__distance.stretch(250, 255)
        #  identify if there's a blob
        self.__blobs = self.__segmented.findBlobs(minsize=2000)

    def get_image(self):
        self.image = self.cam.getImage().flipHorizontal()

    def show_image(self):
        self.image.addDrawingLayer(self.__scope_layer)
        self.image.show()

    def mark_sun(self):
        self.image.drawCircle(
            (self.__circles[-1].x, self.__circles[-1].y),
            self.__circles[-1].radius(),
            Color.BLUE, 3
        )
    
    @property
    def get_sun_coordinates(self):
        return (self.__circles[-1].x, self.__circles[-1].y)
                            
    def initialize_scope_layer(self):
        self.__scope_layer = DrawingLayer(self.window_size) #  same as window size
        #  (position/coordinates, diameter, Color, Thickness of the lines)
        self.__scope_layer.circle(self.__window_center, 50, Color.BLACK, width=3)
        self.__scope_layer.circle(self.__window_center, 100, Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0], self.__window_center[1]-50),(self.__window_center[0], 0), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0], self.__window_center[1]+50), (self.__window_center[0], self.window_size[1]), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0]-50, self.__window_center[1]), (0, self.__window_center[1]), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0]+50, self.__window_center[1]), (self.window_size[0], self.__window_center[1]), Color.BLACK, width=2)
            
    def print_sun_coordinates(self):
        if self.__circles:
            print("x:", self.get_sun_coordinates[0], "y:", self.get_sun_coordinates[1])
    
    @property
    def get_window_center(self):
        return self.__window_center


class SolarMovement(object):
    def __int__(
        self,
        pulneg=11, dirpos=13,
        dirneg=15, enblpin=12
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
        self.servo.set_pwm(0, 0, self.initial)
        time.sleep(0.016667)

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
        self.servo_current = self.servo_current - 1
        self.servo_current = (self.servo_min
                              if self.servo_current <= self.servo_min
                              else self.servo_current)
        self.__servo_move()

    def servo_left(self):
        self.servo_current = self.servo_current + 1
        self.servo_current = (self.servo_max
                              if self.servo_current >= self.servo_max
                              else self.servo_current)
        self.__servo_move()

    def __servo_move(self):
        self.servo.set_pwm(0, 0, self.servo_current)
        time.sleep(0.010)
