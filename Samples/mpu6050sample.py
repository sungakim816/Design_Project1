from mpu6050 import mpu6050
import time
import math

if __name__ == "__main__":
    mpu = mpu6050(0x68)
    while True:
        # print(mpu.get_temp())
        accel_data = mpu.get_accel_data()
        # print(accel_data['x'])
        # print(accel_data['y'])
        # print(accel_data['z'])
        accX = (accel_data['x']) / 16384.0
        accY = (accel_data['y']) / 16384.0
        accZ = (accel_data['z']) / 16384.0
        angleAccX = math.atan2(accY, accZ + abs(accX)) * 360 / 2.0 / math.pi
        angleAccY = math.atan2(accX, accZ + abs(accY)) * 360 / -2.0 / math.pi
        print("Angle at X-Axis: {}".format(angleAccX))
        print("Angle at Y-Axis: {}".format(angleAccY))
        # gyro_data = mpu.get_gyro_data()
        # print(gyro_data['x'])
        # print(gyro_data['y'])
        # print(gyro_data['z'])
        time.sleep(1)
