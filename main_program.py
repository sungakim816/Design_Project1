from SimpleCV import Camera, Color, Display, DrawingLayer
import Adafruit_PCA9685
import spidev
import RPi.GPIO as GPIO
from Tools.RPi_I2C_driver import LiquidCrystal as LiquidCrystalDisplay
from Tools.max6675 import MAX6675
import time

lcd = LiquidCrystalDisplay()

cs_pin = 24
clock_pin = 23
data_pin = 22
unit = "c"
tc = 0.0
thermocouple = MAX6675(cs_pin, clock_pin, data_pin, unit)

while True:
    try:
        cam = Camera(0)
        img = cam.getImage().flipHorizontal()
    except:
        continue
    else:
        break

PULneg = 11
DIRpos = 13
DIRneg = 15
enblPin = 12
counter = 90
state = True
sensitivity = 10


def move_step():
    global counter
    global state
    global PULneg

    if counter != 0 or counter != 180:
        if state:
            counter = counter + 1
        else:
            counter = counter - 1

        for step in range(0, 67, 1):
            GPIO.output(PULneg, True)
            time.sleep(0.00035)
            GPIO.output(PULneg, False)
            time.sleep(0.00035)


def move_step1():
    global counter
    global state
    global PULneg
    if counter != 0 or counter != 360:
        if state:
            counter = counter + 2
        else:
            counter = counter - 2
        for step in range(0, 134, 1):
            GPIO.output(PULneg, True)
            time.sleep(0.00035)
            GPIO.output(PULneg, False)
            time.sleep(0.00035)


def move_right():
    global state
    global DIRpos
    global DIRneg
    GPIO.output(DIRpos, True)
    GPIO.output(DIRneg, False)
    state = True


def move_left():
    global state
    global DIRpos
    global DIRneg
    GPIO.output(DIRpos, False)
    GPIO.output(DIRneg, True)
    state = False


GPIO.setmode(GPIO.BOARD)
GPIO.setup(PULneg, GPIO.OUT)
GPIO.setup(DIRpos, GPIO.OUT)
GPIO.setup(DIRneg, GPIO.OUT)
GPIO.setup(enblPin, GPIO.OUT)

GPIO.output(PULneg, False)
GPIO.output(DIRpos, False)
GPIO.output(DIRneg, False)
GPIO.output(enblPin, True)

########################################################################################################################

# CV Initialization
winsize = (640, 480)
display = Display(winsize)
normaldisplay = True

# SERVO INITIALIZATION
pwm = Adafruit_PCA9685.PCA9685(0x40)  # PCA
servo_initial = 375
circle_x = 0
circle_y = 0
servo_min = 125  # Min pulse length out of 4096
servo_max = 625  # Max pulse length out of 4096


# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

pwm.set_pwm(0, 0, servo_initial)
time.sleep(0.0166667)
i = servo_initial

# dito i-uudjust ung center for calibration ng fresnel lens
center = (320, 240)
scope_layer = DrawingLayer(winsize)  # same as window size


#  (position/coordinates, diameter, Color, Thickness of the lines)
scope_layer.circle(center, 50, Color.BLACK, width=3)
scope_layer.circle(center, 100, Color.BLACK, width=2)
scope_layer.line((center[0], center[1] - 50),
                 (center[0], 0), Color.BLACK, width=2)
scope_layer.line((center[0], center[1] + 50),
                 (center[0], winsize[1]), Color.BLACK, width=2)
scope_layer.line((center[0]-50, center[1]),
                 (0, center[1]), Color.BLACK, width=2)
scope_layer.line((center[0]+50, center[1]),
                 (winsize[0], center[1]), Color.BLACK, width=2)


def temp_reading():
    temp = thermocouple.get()
    lcd.lcd_display_string("Temperature: ", 1)
    lcd.lcd_display_string(str(temp), 2)


try:
    while display.isNotDone():
        while True:
            temp_reading()
            img = cam.getImage().flipHorizontal()
            dist = img.colorDistance(Color.BLACK).dilate(2)
            segmented = dist.stretch(250, 255)
            blobs = segmented.findBlobs(minsize=2000)

            if blobs:
                circles = blobs.filter([b.isCircle(0.2) for b in blobs])
                if circles:
                    img.drawCircle(
                        (circles[-1].x, circles[-1].y), circles[-1].radius(), Color.BLUE, 3)
                    print("Sun Found")
                    print(circles[-1].x)
                    print(circles[-1].y)
                    img.addDrawingLayer(scope_layer)
                    img.show()
                    break

                else:
                    img.addDrawingLayer(scope_layer)
                    img.show()
                    print('Sun Not Found')

            else:
                img.addDrawingLayer(scope_layer)
                img.show()

        # for adjusting the x-axis
        if 320 - circles[-1].x > sensitivity or circles[-1].x - 320 > sensitivity:
            GPIO.output(enblPin, True)
            if circles[-1].x < 320:
                move_left()
                move_step()
            elif circles[-1].x > 320:
                move_right()
                move_step()
        else:
            print('X-axis is centered')
            GPIO.output(enblPin, False)
        # for adjusting the y-axis
        if 240 - circles[-1].y > sensitivity or circles[-1].y - 240 > sensitivity:
            if circles[-1].y < 240:  # to right
                i = i - 1
                pwm.set_pwm(0, 0, i)
                time.sleep(0.010)
                if i < 125:
                    i = 125
            elif circles[-1].y > 240:  # to left?
                i = i + 1
                pwm.set_pwm(0, 0, i)
                time.sleep(0.010)
                if i > 625:
                    i = 625
                else:
                    print('Y-axis is Centered')
except KeyboardInterrupt:
    GPIO.cleanup()
    thermocouple.cleanup()
    print('Terminated')
