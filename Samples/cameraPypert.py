from Tools.solar import SolarCamera, SolarMovement
from multiprocessing import Process, Value, Lock
import time


solar_dream = SolarCamera()


def monitor_display():
    try:
        while True:
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                coor = solar_dream.get_sun_coordinates
                print(coor)
                solar_dream.mark_sun()
            solar_dream.show_image()
            print("Camera Thread")
    except KeyboardInterrupt:
        print("Camera Terminated")


def print_hello():
    try:
        while True:
            print("Hello World")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Hello Terminated")


if __name__ == "__main__":
    monitor_display()
    print("End of the Main Thread")
