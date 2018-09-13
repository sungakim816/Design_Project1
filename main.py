from Tools.solar import SolarCamera, SolarMovement
import RPi.GPIO as GPIO
import time
from multiprocessing import Value, Array, Process, Lock, Queue
from Tools.RPi_I2C_driver import LiquidCrystal as LiquidCrystalDisplay
from Tools.max6675 import MAX6675


# pulneg, dirpos, dirneg, enblpin, servo_increment
solar_movement = SolarMovement()
solar_dream = SolarCamera()
lcd = LiquidCrystalDisplay()
cs_pin = 24
clock_pin = 23
data_pin = 22
unit = "c"
thermocouple = MAX6675(cs_pin, clock_pin, data_pin, unit)
current_servo_pos = Value('i', solar_movement.get_servo_current_position())


def display_temperature_reading():
    while True:
        temp = thermocouple.get()
        lcd.lcd_display_string("Temperature:", 1)
        lcd.lcd_display_string("{0} *C".format(str(temp)), 2)
        time.sleep(1)
        lcd.lcd_clear()


def monitor_display():
    solar_dream.get_image()
    if solar_dream.is_there_sun:
        solar_dream.mark_sun()
    solar_dream.show_image()


def automated(sensitivity, auto):
    global current_servo_pos
    sun_coor = None
    if solar_dream.is_there_sun:
        sun_coor = solar_dream.get_sun_coordinates
        solar_dream.mark_sun()
    #  X-axis
    solar_movement.stepper_enable()  # enable stepper motor
    while sun_coor and abs(window_center[0] - sun_coor[0]) > sensitivity and auto.value:
        print("Adjusting Stepper")
        if sun_coor[0] < window_center[0]:
            solar_movement.stepper_move_left(67)
        elif sun_coor[0] > window_center[0]:
            solar_movement.stepper_move_right(67)
        solar_dream.get_image()  # get new image
        if solar_dream.is_there_sun:
            sun_coor = solar_dream.get_sun_coordinates
            solar_dream.mark_sun()
        else:
            sun_coor = None
        solar_dream.show_image()
        auto.value = GPIO.input(switch_auto)
    solar_movement.stepper_disable()  # disable stepper motor
    # Y-axis
    solar_movement.set_servo_current_position(current_servo_pos.value)
    while sun_coor and abs(window_center[1] - sun_coor[1]) > sensitivity and auto.value:
        print("Adjusting Servo")
        if sun_coor[1] < window_center[1]:
            solar_movement.servo_right()

        elif sun_coor[1] > window_center[1]:
            solar_movement.servo_left()
        solar_dream.get_image()  # get new image
        if solar_dream.is_there_sun:
            sun_coor = solar_dream.get_sun_coordinates
            solar_dream.mark_sun()
        else:
            sun_coor = None
        solar_dream.show_image()  # show the image
        auto.value = GPIO.input(switch_auto)
        current_servo_pos.value = solar_movement.get_servo_current_position()


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)


def manualStepperAdjust(manual):
    currentStepperLeft = False
    currentStepperRight = False
    previousStepperLeft = False
    previousStepperRight = False

    while manual.value:
        currentStepperLeft = read_debounce(
            input_stepper_left, previousStepperLeft)
        currentStepperRight = read_debounce(
            input_stepper_right, previousStepperRight)
        if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_left(200)
            print('Stepper Moving Left')
            solar_movement.stepper_disable()
        elif (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_right(200)
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


def standbyMode():
    monitor_display()


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
    window_center = solar_dream.get_window_center
    sensitivity = 10
    # lock = Lock()
    try:
        process0 = Process(target=display_temperature_reading)
        process0.start()
        while True:
            auto = Value('i', GPIO.input(switch_auto))
            manual = Value('i', GPIO.input(switch_manual))
            if auto.value:
                print("Auto Mode")
                solar_dream.get_image()
                automated(sensitivity, auto)
                solar_dream.show_image()
            elif manual.value:
                print("Manual Mode")
                process1 = Process(target=manualServoAdjust,
                                   args=(manual, ))
                process2 = Process(target=manualStepperAdjust,
                                   args=(manual, ))
                process1.start()
                process2.start()
                process1.join()
                process2.join()
            else:
                solar_movement.stepper_disable()
                print("StandBy Mode")
                standbyMode()
        process0.join()
    except KeyboardInterrupt:
        lcd.lcd_clear()
        solar_movement.clean_up()
        thermocouple.cleanup()
        print('Terminated')
