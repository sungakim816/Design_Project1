import RPi.GPIO as GPIO
import time
from Tools.solar import SolarMovement

solar_movement = SolarMovement()
solar_movement.set_servo_increment(3)

previousServoLeft = False
previousServoRight = False
previousStepperLeft = False
previousStepperRight = False

currentServoLeft = False
currentServoRight = False
currentStepperLeft = False
currentStepperRight = False


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)
        

def main():
    global previousServoLeft
    global previousServoRight
    global previousStepperLeft
    global previousStepperRight

    global currentServoLeft
    global currentServoRight 
    global currentStepperLeft
    global currentStepperRight

    stateServoLeft = False
    stateServoRight = False
    stateStepperLeft = False
    stateStepperRight = False
    
    GPIO.setmode(GPIO.BOARD)
    
    input_servo_left = 31
    input_servo_right = 33
    input_stepper_left = 35
    input_stepper_right = 37
    # output_servo_left = 40
    # output_servo_right = 38
    # output_stepper_left = 36
    # output_stepper_right = 32
    
    GPIO.setup(input_servo_left, GPIO.IN)
    GPIO.setup(input_servo_right, GPIO.IN)
    GPIO.setup(input_stepper_left, GPIO.IN)
    GPIO.setup(input_stepper_right, GPIO.IN)
    # GPIO.setup(output_servo_left, GPIO.OUT)
    # GPIO.setup(output_servo_right, GPIO.OUT)
    # GPIO.setup(output_stepper_left, GPIO.OUT)
    # GPIO.setup(output_stepper_right, GPIO.OUT)
    conditionStepperLeft = True
    conditionStepperRight = True
    
    try:
        while True:
            currentServoLeft = read_debounce(input_servo_left, previousServoLeft)
            currentServoRight = read_debounce(input_servo_right, previousServoRight)
            currentStepperLeft = read_debounce(input_stepper_left, previousStepperLeft)
            currentStepperRight = read_debounce(input_stepper_right, previousStepperRight)

            if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
                solar_movement.servo_left()
            previousServoLeft = currentServoLeft

            if previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
                solar_movement.servo_right()
            previousServoRight = currentServoRight

            if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
                solar_movement.stepper_enable()
                solar_movement.stepper_move_left(300)
                print('Left')
                solar_movement.stepper_disable()
            previousStepperLeft = currentStepperLeft

            if (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
                solar_movement.stepper_enable()
                solar_movement.stepper_move_right(300)
                print('Right')
                solar_movement.stepper_disable()
            previousStepperRight = currentStepperRight
                          
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    solar_movement.clean_up()
