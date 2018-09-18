from Tools.solar import SolarCamera, SolarMovement
import RPi.GPIO as GPIO
import time
from multiprocessing import Value, Array, Process
from Tools.RPi_I2C_driver import LiquidCrystal as LiquidCrystalDisplay
from Tools.max6675 import MAX6675
from mpu6050 import mpu6050
import math
from subprocess import call
from datetime import datetime


# pulneg, dirpos, dirneg, enblpin, servo_increment
solar_movement = SolarMovement()
solar_dream = SolarCamera()
display = LiquidCrystalDisplay()
# cs_pin clock_pin, data_pin, unit
thermocouple = MAX6675(24, 23, 22, 'c')
mpuSensor = mpu6050(0x68)
NOW = datetime.now()
# do not forget, the progam will be terminated on exactly 5PM everyday
TIME_LIMIT = datetime(NOW.year, NOW.month, NOW.day, 23, 59)
INPUT_SERVO_LEFT = 31
INPUT_SERVO_RIGHT = 33
INPUT_STEPPER_LEFT = 35
INPUT_STEPPER_RIGHT = 37
GPIO.setmode(GPIO.BOARD)
GPIO.setup(INPUT_SERVO_LEFT, GPIO.IN)
GPIO.setup(INPUT_SERVO_RIGHT, GPIO.IN)
GPIO.setup(INPUT_STEPPER_LEFT, GPIO.IN)
GPIO.setup(INPUT_STEPPER_RIGHT, GPIO.IN)
GPIO.setup(36, GPIO.IN)  # MASTER SOFTWARE SWITCH
GPIO.setup(40, GPIO.IN)  # MANUAL SWITCH
GPIO.setup(38, GPIO.IN)  # AUTO SWITCH
global_sun_coor = Array('i', 2)
current_stepper_angle = Value('d', 90.00)
current_servo_pos = Value('i', solar_movement.get_servo_current_position())
WINDOW_CENTER = solar_dream.get_window_center
SENSITIVITY = 10
MPU_SENSITIVITY = 2.5
SERVO_SEARCH_PATTERN = (375, 291, 250, 458, 500)
STEPPER_SEARCH_PATTERN = (0, 30, 60, -30, -60)
master = Value('i', GPIO.input(36))
auto = Value('i', GPIO.input(40))
manual = Value('i', GPIO.input(38))
master.value = 1


def read_mode():
    global auto
    global manual
    global master
    while datetime.now() <= TIME_LIMIT:
        master.value = 1  # DO NOT FORGET ABOUT THIS LINE OF CODE
        auto.value = GPIO.input(40)
        manual.value = GPIO.input(38)


def lcd_display_temp_mode():
    while datetime.now() <= TIME_LIMIT and master.value:
        temp = thermocouple.get()
        display.lcd_display_string("Temp: {0}*C".format(str(temp)), 1)
        if auto.value:
            display.lcd_display_string("Mode: Auto", 2)
            time.sleep(1)
            display.lcd_clear()
            display.lcd_display_string("Sun Coordinates", 1)
            display.lcd_display_string("{}".format(global_sun_coor[:]), 2)
            time.sleep(1)
            display.lcd_clear()
        elif manual.value:
            display.lcd_display_string("Mode: Manual", 2)
            time.sleep(1)
            display.lcd_clear()
        else:
            display.lcd_display_string("Mode: StandBy", 2)
            time.sleep(1)
            display.lcd_clear()
            display.lcd_display_string("Sun Coordinates", 1)
            display.lcd_display_string("{}".format(global_sun_coor[:]), 2)
            time.sleep(1)
            display.lcd_clear()


def get_angle_from_mpuSensor():
    accel_data = mpuSensor.get_accel_data()
    accX = (accel_data['x']) /16384.0
    accY = (accel_data['y']) / 16384.0
    accZ = (accel_data['z']) / 16384.0
    angleAccX = math.atan2(accY, accZ + abs(accX)) * 360 / 2.0 / math.pi
    return angleAccX


def monitor_display():
    solar_dream.get_image()
    if solar_dream.is_there_sun:
        solar_dream.mark_sun()
        global_sun_coor[:] = solar_dream.get_sun_coordinates
    else:
        global_sun_coor[:] = [0, 0]
    solar_dream.show_image()


def servo_search_move(pos):
    global current_servo_pos
    current_servo_pos.value = SERVO_SEARCH_PATTERN[pos]
    solar_movement.set_servo_current_position(current_servo_pos.value)


def stepper_search_move(pos):
    solar_movement.stepper_enable()  # enable stepper motor
    if current_stepper_angle.value > pos:
        while abs(abs(current_stepper_angle.value) - abs(STEPPER_SEARCH_PATTERN[pos])) > MPU_SENSITIVITY and auto.value:
            solar_movement.stepper_move_left()
    elif current_stepper_angle.value < pos:
        while abs(abs(current_stepper_angle.value) - abs(STEPPER_SEARCH_PATTERN[pos])) > MPU_SENSITIVITY and auto.value:
            solar_movement.stepper_move_right()
    else:
        pass  # do nothing
    solar_movement.stepper_disable()  # disable stepper motor


def searching_for_sun(auto):
    servo_pos_count = 0
    stepper_pos_count = 0
    solar_movement.set_servo_current_position(current_servo_pos.value)
    solar_dream.get_image()
    while not solar_dream.is_there_sun and auto.value:
        while servo_pos_count < 5 and not solar_dream.is_there_sun and auto.value:
            servo_search_move(servo_pos_count)
            servo_pos_count = servo_pos_count + 1
            print('Searching...')
            time.sleep(1)
            solar_dream.get_image()
            solar_dream.show_image()
            solar_dream.is_there_sun
        servo_pos_count = 0
        stepper_search_move(stepper_pos_count)
        stepper_pos_count = stepper_pos_count + \
            1 if stepper_pos_count < 5 else 0


def automated(SENSITIVITY, auto):
    global current_servo_pos
    global current_stepper_angle
    global global_sun_coor
    global_sun_coor[:] = [0, 0]
    if solar_dream.is_there_sun:
        global_sun_coor[:] = solar_dream.get_sun_coordinates
        solar_dream.mark_sun()
        #  X-axis Stepper
        print('Tracking the Sun...')
        solar_movement.stepper_enable()
        current_stepper_angle.value = get_angle_from_mpuSensor()
        while global_sun_coor[0] != 0 and (abs(WINDOW_CENTER[0] - global_sun_coor[0]) > SENSITIVITY) and (abs(current_stepper_angle.value) <= 60) and auto.value:
            print("Adjusting Stepper")
            if global_sun_coor[0] < WINDOW_CENTER[0]:
                solar_movement.stepper_move_left(67)
            elif global_sun_coor[0] > WINDOW_CENTER[0]:
                solar_movement.stepper_move_right(67)
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                global_sun_coor[:] = solar_dream.get_sun_coordinates
                solar_dream.mark_sun()
            else:
                global_sun_coor[:] = [0, 0]
            current_stepper_angle.value = get_angle_from_mpuSensor()
            solar_dream.show_image()
        solar_movement.stepper_disable()
        # Y-axis Servo
        solar_movement.set_servo_current_position(current_servo_pos.value)
        while global_sun_coor[1] != 0 and abs(WINDOW_CENTER[1] - global_sun_coor[1]) > SENSITIVITY and auto.value:
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
                global_sun_coor[:] = [0, 0]
            solar_dream.show_image()
            current_servo_pos.value = solar_movement.get_servo_current_position()
        print('Centered...')
        # time.sleep(600)
    else:
        searching_for_sun(auto)


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)


def manualStepperAdjust(manual):
    print('Manual Stepper')
    currentStepperLeft = False
    currentStepperRight = False
    previousStepperLeft = False
    previousStepperRight = False
    current_stepper_angle.value = get_angle_from_mpuSensor()
    while manual.value and abs(current_stepper_angle.value) <= 60 and master.value:
        currentStepperLeft = read_debounce(
            INPUT_STEPPER_LEFT, previousStepperLeft)
        currentStepperRight = read_debounce(
            INPUT_STEPPER_RIGHT, previousStepperRight)
        solar_movement.stepper_enable()
        if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
            solar_movement.stepper_move_left(67)
            print('Stepper Moving Left')
        elif (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
            solar_movement.stepper_move_right(67)
            print('Stepper Moving Right')
        solar_movement.stepper_disable()
        previousStepperLeft = currentStepperLeft
        previousStepperRight = currentStepperRight
        current_stepper_angle.value = get_angle_from_mpuSensor()


def manualServoAdjust(manual):
    print('Manual Servo')
    global current_servo_pos
    previousServoLeft = False
    previousServoRight = False
    currentServoLeft = False
    currentServoRight = False
    solar_movement.set_servo_increment(1)
    solar_movement.set_servo_current_position(current_servo_pos.value)
    while manual.value and master.value:
        currentServoLeft = read_debounce(INPUT_SERVO_LEFT, previousServoLeft)
        currentServoRight = read_debounce(
            INPUT_SERVO_RIGHT, previousServoRight)
        if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
            solar_movement.servo_left()
            print('Servo Moving Left')
        elif previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
            solar_movement.servo_right()
            print('Servo Moving Right')
        previousServoLeft = currentServoLeft
        previousServoRight = currentServoRight
    current_servo_pos.value = solar_movement.get_servo_current_position()


if __name__ == "__main__":
    try:
        switchModeThread = Process(target=read_mode)
        lcdThread = Process(target=lcd_display_temp_mode)
        mpuSensorThread = Process(target=get_angle_from_mpuSensor)
        switchModeThread.start()
        lcdThread.start()
        mpuSensorThread.start()
        print("Program Running...")
        while datetime.now() <= TIME_LIMIT and master.value:
            if auto.value:
                solar_dream.get_image()
                automated(SENSITIVITY, auto)

            elif manual.value:
                print("Manual Mode")
                servoAdjustThread = Process(
                    target=manualServoAdjust, args=(manual, ))
                stepperAdjustThread = Process(
                    target=manualStepperAdjust, args=(manual, ))
                servoAdjustThread.start()
                stepperAdjustThread.start()
                servoAdjustThread.join()
                stepperAdjustThread.join()
            else:
                solar_movement.stepper_disable()
                monitor_display()
        switchModeThread.join()
        lcdThread.join()
        mpuSensorThread.join()
        display.lcd_clear()
        GPIO.cleanup()
        print('Main Thread Terminated')
        call("pkill python", shell=True)  # kill python program
    except KeyboardInterrupt:
        display.lcd_clear()
        GPIO.cleanup()
        print('Main Thread Terminated')
        call("pkill python", shell=True)  # kill python program
