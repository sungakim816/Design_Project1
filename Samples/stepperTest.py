import RPi.GPIO as GPIO
import time

PULneg = 11
DIRpos = 13
DIRneg = 15
enblPin = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PULneg, GPIO.OUT)
GPIO.setup(DIRpos, GPIO.OUT)
GPIO.setup(DIRneg, GPIO.OUT)
GPIO.setup(enblPin, GPIO.OUT)

GPIO.output(PULneg, False)
GPIO.output(DIRpos, False)
GPIO.output(DIRneg, False)
GPIO.output(enblPin, True)


def move_step(n):
    for step in range(n):
        GPIO.output(PULneg, True)
        time.sleep(0.00035)
        GPIO.output(PULneg, False)
        time.sleep(0.00035)


def move_right():
    GPIO.output(DIRpos, True)
    GPIO.output(DIRneg, False)


def move_left():
    GPIO.output(DIRpos, False)
    GPIO.output(DIRneg, True)


def main():
    while True:
        move_left()
        move_step(800)
        time.sleep(1)
        # move_right()
        # move_step(800)
        # time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
