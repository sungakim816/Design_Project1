from Tools.solar import SolarCamera, SolarMovement


solar_dream = SolarCamera()
solar_movement = SolarMovement()

switch_auto = 40
switch_manual = 38
switch_standBy = 32
switch_halt = 36

GPIO.setup(switch_auto, GPIO.IN)
GPIO.setup(switch_manual, GPIO.IN)
GPIO.setup(switch_halt, GPIO.IN)

def automated(sensitivity):            
    #  X-axis
    while abs(window_center[0] - sun_coor[0]) > sensitivity:
        solar_movement.stepper_enable()
        if sun_coor[0] < window_center[0]:
            solar_movement.stepper_move_left()
        elif sun_coor[0] > window_center[0]:
            solar_movement.stepper_move_right()
    solar_movement.stepper_disable()
    print('X-axis is centered!')
            
    #  Y-axis
    while abs(window_center[1] - sun_coor[1]) > sensitivity:
        if sun_coor[1] < window_center[1]:
            solar_movement.servo_right()
        elif sun_coor[1] > window_center[1]:
            solar_movement.servo_left()
    print('Y-axis is centered')


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
        

def manual():
    global solar_movement
    solar_movement.set_servo_increment(3) # increase servo step
    global previousServoLeft
    global previousServoRight
    global previousStepperLeft
    global previousStepperRight

    global currentServoLeft
    global currentServoRight 
    global currentStepperLeft
    global currentStepperRight
    
    GPIO.setmode(GPIO.BOARD)
    
    input_servo_left = 31
    input_servo_right = 33
    input_stepper_left = 35
    input_stepper_right = 37
    
    GPIO.setup(input_servo_left, GPIO.IN)
    GPIO.setup(input_servo_right, GPIO.IN)
    GPIO.setup(input_stepper_left, GPIO.IN)
    GPIO.setup(input_stepper_right, GPIO.IN)
    
    while GPIO.input(switch_manual):
        currentServoLeft = read_debounce(input_servo_left, previousServoLeft)
        currentServoRight = read_debounce(input_servo_right, previousServoRight)
        currentStepperLeft = read_debounce(input_stepper_left, previousStepperLeft)
        currentStepperRight = read_debounce(input_stepper_right, previousStepperRight)

        if (previousServoLeft == False and currentServoLeft) or (previousServoLeft and currentServoLeft):
            # solar_movement.servo_left()
            print("Servo Moving Left")
        previousServoLeft = currentServoLeft

        if previousServoRight == False and currentServoRight or (previousServoRight and currentServoRight):
            # solar_movement.servo_right()
            print("Servo Moving Right")
        previousServoRight = currentServoRight

        if (previousStepperLeft == False and currentStepperLeft) or (previousStepperLeft and currentStepperLeft):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_left(300)
            print('Stepper Moving Left')
            solar_movement.stepper_disable()
        previousStepperLeft = currentStepperLeft

        if (previousStepperRight == False and currentStepperRight) or (previousStepperRight and currentStepperRight):
            solar_movement.stepper_enable()
            solar_movement.stepper_move_right(300)
            print('Stepper Moving Right')
            solar_movement.stepper_disable()
        previousStepperRight = currentStepperRight

    previousServoLeft = False
    previousServoRight = False
    previousStepperLeft = False
    previousStepperRight = False

    currentServoLeft = False
    currentServoRight = False
    currentStepperLeft = False
    currentStepperRight = False
    solar_movement.set_servo_increment(1) # switch to normal minimum sensitivity
    GPIO.cleanup() # Clean all used GPIO pins

def main():
    sensitivity = 10
    try:
        while True:
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                solar_dream.mark_sun()
                sun_coor = solar_dream.get_sun_coordinates
                window_center = solar_dream.get_window_center
            else:
                print("Sun Not Found")

            if GPIO.input(switch_auto):
                automated(sensitivity)
            elif GPIO.input(switch_manual):
                manual()
            else:
                pass
    
    except KeyboardInterrupt:
        solar_movement.clean_up()
        print('Terminated')


if __name__ == "__main__":
    #  main thread
    main()
