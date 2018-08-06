from SimpleCV import Camera, Color, Display, DrawingLayer
import Adafruit_PCA9685
import spidev
import RPi.GPIO as GPIO
from Tools.RPi_I2C_driver import lcd as LiquidCrystalDisplay
from Tools.max6675 import MAX6675
import time

while True:
    try:
        cam = Camera(0)
    except:
        continue
    else:
        break
# CV Initialization
image = cam.getImage().flipHorizontal()
winsize = (640, 480)
display = Display(winsize)
normaldisplay = True

center = (320,240) #  Central Coordinates
scope_layer = DrawingLayer(winsize) #  same as window size

#  (position/coordinates, diameter, Color, Thickness of the lines)
scope_layer.circle(center, 50, Color.BLACK, width=3)
scope_layer.circle(center, 100, Color.BLACK, width=2)
scope_layer.line((center[0], center[1] - 50),(center[0], 0), Color.BLACK, width=2)
scope_layer.line((center[0],center[1] + 50), (center[0], winsize[1]), Color.BLACK, width=2)
scope_layer.line((center[0]-50, center[1]), (0, center[1]), Color.BLACK, width=2)
scope_layer.line((center[0]+50, center[1]), (winsize[0], center[1]), Color.BLACK, width=2)

try:
    while display.isNotDone():
        while True:
            image = cam.getImage().flipHorizontal()
            distance = image.colorDistance(Color.BLACK).dilate(2)
            segmented = distance.stretch(250, 255)
            blobs = segmented.findBlobs(minsize=2000)
            if blobs:
                circles = blobs.filter([b.isCircle(0.2) for b in blobs])
                if circles:
                    image.drawCircle((circles[-1].x, circles[-1].y), circles[-1].radius(), Color.BLUE, 3)
                    print("Sun Found")
                    print(circles[-1].x)
                    print(circles[-1].y)
                    image.addDrawingLayer(scope_layer)
                    image.show()
                    break
                
                else:
                    image.addDrawingLayer(scope_layer)
                    image.show()
                    print('Sun Not Found')
            
            else:
                image.addDrawingLayer(scope_layer)
                image.show()
except KeyboardInterrupt:
    GPIO.cleanup()
    print('Terminated')

