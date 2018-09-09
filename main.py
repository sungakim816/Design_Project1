from Tools.solar import SolarCamera, SolarMovement
import RPi.GPIO as GPIO


solar_dream = SolarCamera()
# pulneg, dirpos, dirneg, enblpin, servo_increment
solar_movement = SolarMovement(11, 13, 15, 12, 1)

window_center = solar_dream.get_window_center
sun_coor = None

previousServoLeft = False
previousServoRight = False
previousStepperLeft = False
previousStepperRight = False

currentServoLeft = False
currentServoRight = False
currentStepperLeft = False
currentStepperRight = False

input_servo_left = 31
input_servo_right = 33
input_stepper_left = 35
input_stepper_right = 37

switch_auto = 40
switch_manual = 38
switch_standBy = 32
switch_halt = 36

GPIO.setmode(GPIO.BOARD)
    
GPIO.setup(input_servo_left, GPIO.IN)
GPIO.setup(input_servo_right, GPIO.IN)
GPIO.setup(input_stepper_left, GPIO.IN)
GPIO.setup(input_stepper_right, GPIO.IN)
GPIO.setup(switch_auto, GPIO.IN)
GPIO.setup(switch_manual, GPIO.IN)
GPIO.setup(switch_halt, GPIO.IN)

def automated(sensitivity):
    global window_center
    global sun_coor
    if solar_dream.is_there_sun:
        solar_dream.mark_sun()
        sun_coor = solar_dream.get_sun_coordinates
    #  X-axis
    solar_movement.stepper_enable()
    while sun_coor is not None and abs(window_center[0] - sun_coor[0]) > sensitivity:
        if sun_coor[0] < window_center[0]:
            solar_movement.stepper_move_left()
        elif sun_coor[0] > window_center[0]:
            solar_movement.stepper_move_right()
        solar_dream.get_image() # get new image
        if solar_dream.is_there_sun:
            sun_coor = solar_dream.get_sun_coordinates
            solar_dream.mark_sun()
        solar_dream.show_image()
    solar_movement.stepper_disable()
    #  Y-axis
    while sun_coor is not None and abs(window_center[1] - sun_coor[1]) > sensitivity:
        if sun_coor[1] < window_center[1]:
            solar_movement.servo_right()
        elif sun_coor[1] > window_center[1]:
            solar_movement.servo_left()
        solar_dream.get_image() # get new image
        if solar_dream.is_there_sun:
            sun_coor = solar_dream.get_sun_coordinates
            solar_dream.mark_sun()
        solar_dream.show_image() # show the image
    solar_dream.show_image()


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)
        
def manualStepperAdjust():
    global currentStepperLeft
    global currentStepperRight
    global previousStepperLeft
    global previousStepperRight
    
    currentServoLeft = read_debounce(input_servo_left, previousServoLeft)
    currentServoRight = read_debounce(input_servo_right, previousServoRight)

    if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
        # solar_movement.stepper_enable()
        # solar_movement.stepper_move_left(300)
        print('Stepper Moving Left')
        # solar_movement.stepper_disable()
    elif (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
        # solar_movement.stepper_enable()
        # solar_movement.stepper_move_right(300)
        print('Stepper Moving Right')
        # solar_movement.stepper_disable()
    previousStepperLeft = currentStepperLeft
    previousStepperRight = currentStepperRight


def manualServoAdjust():
    global previousServoLeft
    global previousServoRight
    global currentServoLeft
    global currentServoRight

    currentStepperLeft = read_debounce(input_stepper_left, previousStepperLeft)
    currentStepperRight = read_debounce(input_stepper_right, previousStepperRight)
    
    if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
        # solar_movement.servo_left()
        print('Servo Moving Left')
    elif previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
        # solar_movement.servo_right()
        print('Servo Moving Right')

    previousServoLeft = currentServoLeft
    previousServoRight = currentServoRight

def manual():
    global solar_movement
    solar_movement.set_servo_increment(3) # increase servo step
    while GPIO.input(switch_auto):
        manualServoAdjust()
        manualStepperAdjust()
    solar_movement.set_servo_increment(1) # switch to normal minimum sensitivity


def main():
    sensitivity = 10
    window_center = solar_dream.get_window_center
    try:
        while True:
            solar_dream.get_image() # get image
            automated(sensitivity)
            solar_dream.show_image()    
    except KeyboardInterrupt:
        solar_movement.clean_up()
        print('Terminated')


if __name__ == "__main__":
    #  main thread
    main()
