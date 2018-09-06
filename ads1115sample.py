import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15

# map function similar to arduino 
def valmap(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

current_value = previous_value = 0

def stepper_direction_movement():
    global current_value
    global previous_value
    direction = None
    current_value = adc.read_adc(1)
    if abs(current_value-previous_value) >= 100:
        if current_value > previous_value:
            direction = "Right" 
        elif current_value < previous_value:
            direction = "Left"
    else:
        direction = "Steady"
    previous_value = current_value
    return direction

try:
    while True:
        servo_map_value = valmap(adc.read_adc(0), 0, 31767, 126, 625 )
        print('Channel 0:', servo_map_value, 'Channel 1:', stepper_direction_movement())
        time.sleep(1)
except KeyboardInterrupt:
    adc.stop_adc()
