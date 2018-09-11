from Tools.solar import SolarCamera, SolarMovement
import RPi.GPIO as GPIO
import time
from multiprocessing import Value, Array, Process
from Tools.RPi_I2C_driver import lcd as LiquidCrystalDisplay
from Tools.max6675 import MAX6675


# pulneg, dirpos, dirneg, enblpin, servo_increment
solar_movement = SolarMovement()
solar_dream = SolarCamera()
lcd = LiquidCrystalDisplay
thermocouple = MAX6675(cs_pin, clock_pin, data_pin, unit)


def display_temperature_reading():
    temp = thermocouple.get()
    lcd.lcd_display_string("Temperature: ", 1)
    lcd.lcd_display_string(str(temp), 2)


def monitor_display():
    solar_dream.get_image()
    if solar_dream.is_there_sun:
        solar_dream.mark_sun()
    solar_dream.show_image()


def monitor_display_manual_mode():
    while manual.value:
        solar_dream.get_image()
        if solar_dream.is_there_sun:
            solar_dream.mark_sun()
        solar_dream.show_image()
        manual = Value("i", GPIO.input(switch_manual))


def automated(sensitivity):
    sun_coor = None
    if solar_dream.is_there_sun:
        sun_coor = solar_dream.get_sun_coordinates
        solar_dream.mark_sun()
    #  X-axis
    solar_movement.stepper_enable()  # enable stepper motor
    while sun_coor and abs(window_center[0] - sun_coor[0]) > sensitivity:
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
    solar_movement.stepper_disable()  # disable stepper motor
    # Y-axis
    while sun_coor and abs(window_center[1] - sun_coor[1]) > sensitivity:
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


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)


def manualStepperAdjust(solar_movement, manual):
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
        manual = Value("i", GPIO.input(switch_manual))


def manualServoAdjust(solar_movement, manual):
    previousServoLeft = False
    previousServoRight = False
    currentServoLeft = False
    currentServoRight = False
    solar_movement.set_servo_increment(1)
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
        manual = Value("i", GPIO.input(switch_manual))


def


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

    try:

        while True:
            auto = Value('i', GPIO.input(switch_auto))
            manual = Value('i', GPIO.input(switch_manual))
            if auto.value:
                print("Auto Mode")
                solar_dream.get_image()
                automated(sensitivity)
                solar_dream.show_image()
            elif manual.value:
                process1 = Process(target=manualServoAdjust,
                                   args=(solar_movement, manual))
                process2 = Process(target=manualStepperAdjust,
                                   args=(solar_movement, manual))
                process1.start()
                process2.start()
                process1.terminate()
                process2.terminate()
                process1.join()
                process2.join()
            else:
                solar_movement.stepper_disable()
                print("StandBy Mode")
                standbyMode()

    except KeyboardInterrupt:
        solar_movement.clean_up()
        print('Terminated')
