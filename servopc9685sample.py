from Adafruit_PCA9685 import PCA9685
import time

servo = PCA9685(0x40)
servo.set_pwm_freq(60)
servo_min = 125
servo_max = 625
servo_initial = 375
servo_current = servo_initial

while True:
    for i in range(125, 626):
        servo.set_pwm(0, 0, i)
        time.sleep(0.010)

    for i in range(624, 124, -1):
        servo.set_pwm(0, 0, i)
        time.sleep(0.010)
