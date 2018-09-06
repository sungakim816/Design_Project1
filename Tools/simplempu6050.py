from mpu6050 import mpu6050
from time import sleep

if __name__ == "__main__":
    mpu = mpu6050(0x68)
    print(mpu.get_temp())
    try:
        while True:
            accel_data = mpu.get_accel_data()
            print("Accelerometer Data")
            print(accel_data)
            sleep(1)
            
            gyro_data = mpu.get_gyro_data()
            print("Gyroscope Data x")
            print('x:',gyro_data['x'])
            sleep(1)
            print(" ")
    except KeyboardInterrupt:
        print("Terminated")
