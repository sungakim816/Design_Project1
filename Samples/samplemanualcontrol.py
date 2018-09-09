import RPi.GPIO as GPIO
import time
# from Tools.solar import SolarMovement

# solar_movement = SolarMovement()
# solar_movement.set_servo_increment(3)

previousServoLeft = False
previousServoRight = False
previousStepperLeft = False
previousStepperRight = False

currentServoLeft = False
currentServoRight = False
currentStepperLeft = False
currentStepperRight = False
GPIO.setmode(GPIO.BOARD)
input_servo_left = 31
input_servo_right = 33
input_stepper_left = 35
input_stepper_right = 37
    
GPIO.setup(input_servo_left, GPIO.IN)
GPIO.setup(input_servo_right, GPIO.IN)
GPIO.setup(input_stepper_left, GPIO.IN)
GPIO.setup(input_stepper_right, GPIO.IN)



def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)
        

def manualStepperAdjust():
    global currentStepperLeft
    global currentStepperRight
    global previousStepperLeft
    global previousStepperRight
    
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
    
    if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
        # solar_movement.servo_left()
        print('Servo Moving Left')
    elif previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
        # solar_movement.servo_right()
        print('Servo Moving Right')
    previousServoLeft = currentServoLeft
    previousServoRight = currentServoRight

def manual():
    try:
        while True:
            currentServoLeft = read_debounce(input_servo_left, previousServoLeft)
            currentServoRight = read_debounce(input_servo_right, previousServoRight)
            currentStepperLeft = read_debounce(input_stepper_left, previousStepperLeft)
            currentStepperRight = read_debounce(input_stepper_right, previousStepperRight)
            manualServoAdjust()
            manualStepperAdjust()
                          
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    solar_movement.clean_up()
