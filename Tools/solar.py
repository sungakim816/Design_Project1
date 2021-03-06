from SimpleCV import (
    Camera,
    Color,
    Display,
    DrawingLayer
)
import RPi.GPIO as GPIO
from PCA9685.pca9685 import PCA9685
import time


class SolarCamera:
    def __init__(self, window_size=(640, 480), **kwargs):
        while True:  # Initialize the Camera
            try:
                cam = Camera()
                cam.getImage().flipHorizontal()
            except:
                continue
            else:
                break
        self.cam = cam
        self.image = None
        self.window_size = window_size
        self.display = Display(self.window_size)
        self.__window_center = (338, 377)# (self.window_size[0]/2, self.window_size[1]/2)
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
            self.__circles = self.__blobs.filter(
                [b.isCircle(0.2) for b in self.__blobs])
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
        self.__scope_layer = DrawingLayer(
            self.window_size)  # same as window size
        #  (position/coordinates, diameter, Color, Thickness of the lines)
        self.__scope_layer.circle(
            self.__window_center, 50, Color.BLUE, width=3)
        self.__scope_layer.circle(
            self.__window_center, 100, Color.BLUE, width=2)
        self.__scope_layer.line(
            (self.__window_center[0], self.__window_center[1]-50), (self.__window_center[0], 0), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0], self.__window_center[1]+50), (
            self.__window_center[0], self.window_size[1]), Color.BLACK, width=2)
        self.__scope_layer.line(
            (self.__window_center[0]-50, self.__window_center[1]), (0, self.__window_center[1]), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0]+50, self.__window_center[1]), (
            self.window_size[0], self.__window_center[1]), Color.BLACK, width=2)

    def print_sun_coordinates(self):
        if self.__circles:
            print("x:", self.get_sun_coordinates[0],
                  "y:", self.get_sun_coordinates[1])

    @property
    def get_window_center(self):
        return self.__window_center


class SolarMovement(object):
    def __init__(
        self, pulneg=11,
        dirpos=13, dirneg=15,
        enblpin=12, servo_increment=1
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
        self.GPIO.output(self.__enblpin, False)
        # Servo Motor
        self.servo = PCA9685(0x40)
        self.servo.set_pwm_freq(60)
        self.servo_min = 208
        self.servo_max = 541
        self.servo_initial = 375
        self.__servo_current = self.servo_initial
        self.__servo_increment = int(servo_increment)
        self.servo.set_pwm(0, 0, self.__servo_current)

    def stepper_move_left(self, steps=134):
        self.GPIO.output(self.__dirpos, False)
        self.GPIO.output(self.__dirneg, True)
        self.__stepper_move(int(steps))

    def stepper_move_right(self, steps=134):
        self.GPIO.output(self.__dirpos, True)
        self.GPIO.output(self.__dirneg, False)
        self.__stepper_move(steps)

    def __stepper_move(self, steps=134):
        for step in range(0, steps, 1):
            self.GPIO.output(self.__pulneg, True)
            time.sleep(0.00035)
            self.GPIO.output(self.__pulneg, False)
            time.sleep(0.00035)

    def stepper_enable(self):
        self.GPIO.output(self.__enblpin, True)

    def stepper_disable(self):
        self.GPIO.output(self.__enblpin, False)

    def __servo_move(self, delay=0.010):
        self.servo.set_pwm(0, 0, self.__servo_current)
        time.sleep(delay)

    def servo_right(self, delay=0.010):
        self.__servo_current = self.__servo_current - self.__servo_increment
        self.__servo_current = (self.servo_min
                                if self.__servo_current <= self.servo_min
                                else self.__servo_current)
        self.__servo_move(delay)

    def servo_left(self, delay=0.010):
        self.__servo_current = self.__servo_current + self.__servo_increment
        self.__servo_current = (self.servo_max
                                if self.__servo_current >= self.servo_max
                                else self.__servo_current)
        self.__servo_move(delay)

    def set_servo_increment(self, servo_increment):
        self.__servo_increment = int(servo_increment)

    def clean_up(self):
        self.GPIO.cleanup()
        GPIO.cleanup()

    def get_servo_current_position(self):
        return self.__servo_current

    def set_servo_current_position(self, servo_pos):
        if servo_pos > self.__servo_current:
            limit = servo_pos
            for i in range(self.__servo_current, limit+1):
                self.__servo_current = i
                self.__servo_move()
        elif servo_pos < self.__servo_current:
            start = self.__servo_current
            for i in range(start, servo_pos-1, -1):
                self.__servo_current = i
                self.__servo_move()
