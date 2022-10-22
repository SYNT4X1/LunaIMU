# quick example for how to interact with the MPU-6050 library from github

from mpu6050 import mpu6050 # import

sensor = mpu6050(0x68) # We need a sensor object 
accelerometer_data = sensor.get_accel_data() # Here's how you'd call functions