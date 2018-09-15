from Tools.solar import SolarCamera, SolarMovement
import RPi.GPIO as GPIO
import time
from multiprocessing import Value, Array, Process, Lock, Queue
from Tools.RPi_I2C_driver import LiquidCrystal as LiquidCrystalDisplay
from Tools.max6675 import MAX6675
from mpu6050 import mpu6050
import math
from subprocess import call

# pulneg, dirpos, dirneg, enblpin, servo_increment
solar_movement = SolarMovement()
solar_dream = SolarCamera()
display = LiquidCrystalDisplay()
CS_PIN = 24
CLOCK_PIN = 23
DATA_PIN = 22
UNIT = "c"
thermocouple = MAX6675(CS_PIN, CLOCK_PIN, DATA_PIN, UNIT)
current_stepper_angle = Value('d', 90.00)
global_sun_coor = Array('i', 2)
current_servo_pos = Value('i', solar_movement.get_servo_current_position())
mpuSensor = mpu6050(0x68)
WINDOW_CENTER = solar_dream.get_window_center
SENSITIVITY = 10
MPU_SENSITIVITY = 2.5
SERVO_SEARCH_PATTERN = (375, 291, 208, 458, 541)
STEPPER_SEARCH_PATTERN = (0, 30, 60, -30, -60)
# is_there_sun = Value('i', 0)


def read_mode():
    while True:
        auto.value = GPIO.input(switch_auto)
        manual.value = GPIO.input(switch_manual)
        time.sleep(0.5)


def lcd_display_temp_mode():
    while True:
        temp = thermocouple.get()
        display.lcd_display_string("Temp: {0}*C".format(str(temp)), 1)
        if GPIO.input(switch_auto):
            display.lcd_display_string("Mode: Auto", 2)
        elif GPIO.input(switch_manual):
            display.lcd_display_string("Mode: Manual", 2)
        else:
            display.lcd_display_string("Mode: StandBy", 2)
        time.sleep(1)
        display.lcd_clear()
        display.lcd_display_string("Sun Coordinates", 1)
        display.lcd_display_string("{}".format(global_sun_coor[:]), 2)
        time.sleep(1)
        display.lcd_clear()
    display.lcd_clear()


def get_angle_from_mpuSensor():
    global current_stepper_angle
    while True:
        accel_data = mpuSensor.get_accel_data()
        accX = (accel_data['x']) / 16384.0
        accY = (accel_data['y']) / 16384.0
        accZ = (accel_data['z']) / 16384.0
        angleAccX = math.atan2(accY, accZ + abs(accX)) * 360 / 2.0 / math.pi
        current_stepper_angle.value = angleAccX
        time.sleep(0.1)


def monitor_display():
    solar_dream.get_image()
    if solar_dream.is_there_sun:
        solar_dream.mark_sun()
        global_sun_coor[:] = solar_dream.get_sun_coordinates
    else:
        global_sun_coor[:] = [-1, -1]
    solar_dream.show_image()


def servo_search_move(pat1):
    current_servo_pos.value = SERVO_SEARCH_PATTERN[pat1]
    solar_movement.set_servo_current_position(current_servo_pos.value)


def stepper_search_move(target):
    if current_stepper_angle.value > target:
        while abs(abs(current_stepper_angle.value) - abs(STEPPER_SEARCH_PATTERN[target])) > MPU_SENSITIVITY and auto.value:
            solar_movement.stepper_move_left()
    elif current_stepper_angle.value < target:
        while abs(abs(current_stepper_angle.value) - abs(STEPPER_SEARCH_PATTERN[target])) > MPU_SENSITIVITY and auto.value:
            solar_movement.stepper_move_right()


def searching_for_sun(auto):
    servo_pos_count = 0
    stepper_search_count = 0
    solar_movement.stepper_enable()  # enable stepper motor
    solar_movement.set_servo_current_position(current_servo_pos.value)
    solar_dream.get_image()
    while not solar_dream.is_there_sun() and auto.value:
        while servo_pos_count <= 5 and not solar_dream.is_there_sun() and auto.value:
            servo_search_move(servo_pos_count)
            solar_dream.get_image()
            solar_dream.is_there_sun()
            solar_dream.show_image()
            servo_pos_count = servo_pos_count + 1
        servo_pos_count = 0
        stepper_search_move(stepper_search_count)
        stepper_search_count = stepper_search_count + \
            1 if stepper_search_count < 5 else 0


def automated(SENSITIVITY, auto):
    global current_servo_pos
    global current_stepper_angle
    global global_sun_coor
    global_sun_coor[:] = [-1, -1]
    if solar_dream.is_there_sun:
        global_sun_coor[:] = solar_dream.get_sun_coordinates[:]
        solar_dream.mark_sun()
        #  X-axis Stepper
        solar_movement.stepper_enable()
        while global_sun_coor[0] != -1 and (abs(WINDOW_CENTER[0] - global_sun_coor[0]) > SENSITIVITY) and (abs(current_stepper_angle.value) <= 60) and auto.value:
            print("Adjusting Stepper")
            if global_sun_coor[0] < WINDOW_CENTER[0]:
                solar_movement.stepper_move_left(67)
            elif global_sun_coor[0] > WINDOW_CENTER[0]:
                solar_movement.stepper_move_right(67)
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                global_sun_coor[:] = solar_dream.get_sun_coordinates[:]
                solar_dream.mark_sun()
            else:
                global_sun_coor[:] = [-1, -1]
            solar_dream.show_image()
        solar_movement.stepper_disable()
        # Y-axis Servo
        solar_movement.set_servo_current_position(current_servo_pos.value)
        while global_sun_coor[1] != -1 and abs(WINDOW_CENTER[1] - global_sun_coor[1]) > SENSITIVITY and auto.value:
            print("Adjusting Servo")
            if global_sun_coor[1] < WINDOW_CENTER[1]:
                solar_movement.servo_right()
            elif global_sun_coor[1] > WINDOW_CENTER[1]:
                solar_movement.servo_left()
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                global_sun_coor[:] = solar_dream.get_sun_coordinates[:]
                solar_dream.mark_sun()
            else:
                global_sun_coor[:] = [-1, -1]
            solar_dream.show_image()
            current_servo_pos.value = solar_movement.get_servo_current_position()
        else:
            searching_for_sun(auto)


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)


def manualStepperAdjust(manual):
    currentStepperLeft = False
    currentStepperRight = False
    previousStepperLeft = False
    previousStepperRight = False

    while manual.value and abs(current_stepper_angle.value) <= 60:
        currentStepperLeft = read_debounce(
            input_stepper_left, previousStepperLeft)
        currentStepperRight = read_debounce(
            input_stepper_right, previousStepperRight)
        if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_left(67)
            print('Stepper Moving Left')
            solar_movement.stepper_disable()
        elif (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_right(67)
            print('Stepper Moving Right')
            solar_movement.stepper_disable()
        previousStepperLeft = currentStepperLeft
        previousStepperRight = currentStepperRight
        manual.value = GPIO.input(switch_manual)


def manualServoAdjust(manual):
    global current_servo_pos
    previousServoLeft = False
    previousServoRight = False
    currentServoLeft = False
    currentServoRight = False
    solar_movement.set_servo_increment(1)
    solar_movement.set_servo_current_position(current_servo_pos.value)
    while manual.value:
        currentServoLeft = read_debounce(input_servo_left, previousServoLeft)
        currentServoRight = read_debounce(
            input_servo_right, previousServoRight)
        if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
            solar_movement.servo_left()
            print('Servo Moving Left')
        elif previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
            solar_movement.servo_right()
            print('Servo Moving Right')
        previousServoLeft = currentServoLeft
        previousServoRight = currentServoRight
        manual.value = GPIO.input(switch_manual)
    current_servo_pos.value = solar_movement.get_servo_current_position()


if __name__ == "__main__":
    input_servo_left = 31
    input_servo_right = 33
    input_stepper_left = 35
    input_stepper_right = 37
    switch_auto = 40
    switch_manual = 38
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(input_servo_left, GPIO.IN)
    GPIO.setup(input_servo_right, GPIO.IN)
    GPIO.setup(input_stepper_left, GPIO.IN)
    GPIO.setup(input_stepper_right, GPIO.IN)
    GPIO.setup(switch_auto, GPIO.IN)
    GPIO.setup(switch_manual, GPIO.IN)
    auto = Value('i', GPIO.input(switch_auto))
    manual = Value('i', GPIO.input(switch_manual))
    print("Program Running...")
    try:
        switchModeThread = Process(target=read_mode)
        lcdThread = Process(target=lcd_display_temp_mode)
        mpuSensorThread = Process(target=get_angle_from_mpuSensor)
        lcdThread.start()
        mpuSensorThread.start()
        switchModeThread.start()
        while True:
            if auto.value:
                solar_dream.get_image()
                automated(SENSITIVITY, auto)
                solar_dream.show_image()
            elif manual.value:
                servoAdjustThread = Process(target=manualServoAdjust,
                                            args=(manual, ))
                stepperAdjustThread = Process(target=manualStepperAdjust,
                                              args=(manual, ))
                servoAdjustThread.start()
                stepperAdjustThread.start()
                servoAdjustThread.terminate()
                stepperAdjustThread.terminate()
                servoAdjustThread.join()
                stepperAdjustThread.join()
            else:  # DO NOT FUCKING FORGET THIS LINE OF CODE
                solar_movement.stepper_disable()
                monitor_display()
        lcdThread.join()
        mpuSensorThread.join()
        switchModeThread.join()
    except KeyboardInterrupt:
        display.lcd_clear()
        GPIO.cleanup()
        print('Main Thread Terminated')
        call("pkill python", shell=True)  # kill python program
