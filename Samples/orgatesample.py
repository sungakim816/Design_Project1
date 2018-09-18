import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.OUT)

try:
    while True:
        GPIO.output(8, True)
except KeyboardInterrupt:
    GPIO.cleanup()
