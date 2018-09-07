import RPi.GPIO as GPIO
import time


previousServoX = False
previousServoY = False
previousStepperX = False
previousStepperY = False

currentServoX = False
currentServoY = False
currentStepperX = False
currentStepperY = False


def read_debounce(pinNum, previousButtonState):
    if GPIO.input(pinNum) != previousButtonState:
        time.sleep(0.01)
    return GPIO.input(pinNum)
        

def main():
    global previousServoX
    global previousServoY
    global previousStepperX
    global previousStepperY

    global currentServoX
    global currentServoY 
    global currentStepperX
    global currentStepperY

    stateServoX = False
    stateServoY = False
    stateStepperX = False
    stateStepperY = False
    
    GPIO.setmode(GPIO.BOARD)
    
    input_servo_x = 31
    input_servo_y = 33
    input_stepper_x = 35
    input_stepper_y = 37
    output_servo_x = 40
    output_servo_y = 38
    output_stepper_x = 36
    output_stepper_y = 32
    
    GPIO.setup(input_servo_x, GPIO.IN)
    GPIO.setup(input_servo_y, GPIO.IN)
    GPIO.setup(input_stepper_x, GPIO.IN)
    GPIO.setup(input_stepper_y, GPIO.IN)
    GPIO.setup(output_servo_x, GPIO.OUT)
    GPIO.setup(output_servo_y, GPIO.OUT)
    GPIO.setup(output_stepper_x, GPIO.OUT)
    GPIO.setup(output_stepper_y, GPIO.OUT)
    
    try:
        while True:
            currentServoX = read_debounce(input_servo_x, previousServoX)
            if previousServoX == False and currentServoX:
                stateServoX = not stateServoX
                print('Button State:', stateServoX)
                GPIO.output(output_servo_x, stateServoX)
            previousServoX = currentServoX
                          
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    
