from Tools.solar import SolarCamera, SolarMovement


solar_dream = SolarCamera()
solar_movement = SolarMovement()


def main():
    sensitivity = 10
    try:
        while True:
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                solar_dream.mark_sun()
                sun_coor = solar_dream.get_sun_coordinates
                window_center = solar_dream.get_window_center
                #  X-axis
                if abs(window_center[0] - sun_coor[0]) > sensitivity:
                    solar_movement.stepper_enable()
                    if sun_coor[0] < window_center[0]:
                        solar_movement.stepper_move_left()
                    elif sun_coor[0] > window_center[0]:
                        solar_movement.stepper_move_right()
                else:
                    solar_movement.stepper_disable()
                    print('X-axis is centered!')
                #  Y-axis
                if abs(window_center[1] - sun_coor[1]) > sensitivity:
                    if sun_coor[1] < window_center[1]:
                        solar_movement.servo_right()
                    elif sun_coor[1] > window_center[1]:
                        solar_movement.servo_left()
                else:
                    print('Y-axis is centered')
            else:
                print('Sun Not Found')  # do something about it
            solar_dream.show_image()
    except KeyboardInterrupt:
        solar_movement.GPIO.cleanup()
        solar_movement.servo_software_reset()
        print('Terminated')


#  main thread
main()
