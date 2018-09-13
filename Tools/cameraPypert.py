from Tools.solar import SolarCamera, SolarMovement
from multiprocessing import Process, Value, Lock
import time


def monitor_display():
    try:
        while True:
            solar_dream.get_image()
            if solar_dream.is_there_sun:
                coor = solar_dream.get_sun_coordinates
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
    process0 = Process(target=monitor_display)
    process1 = Process(target=print_hello)
    process0.start()
    process1.start()
    process0.join()
    process1.join()
    print("End of the Main Thread")
    
