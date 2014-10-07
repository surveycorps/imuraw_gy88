#!/usr/bin/env python

from Adafruit_I2C import Adafruit_I2C

import rospy
import sensor_msgs.msg # IMU data type
from std_msgs.msg import Header # Header to contain timestamp information
from geometry_msgs.msg import Vector3 # Standard Vector form for ROS

import time

class IMU : 
    MPU6050_I2C_ADDRESS = 0x68
    HMC5883L_I2C_ADDRESS = 0x1E 
    #TODO: Handle magnometer gain settings, see Adafruit implementation

    #TODO: Ensure conversion factors agree with ROS
    RAW2ACCEL = 16384 # Conversion factor from raw byte value to g's 
    RAW2GYRO = 131 # Conversion factor from raw byte value to rad/s 
    RAW2MAG = 1

    """
    Define Register values
    """

    REG_MAG_CONT_MODE = 0x02
    REG_IMU_RESET = 0x6B 

    REG_ACCEL_X_HI = 0x3b
    REG_ACCEL_X_LOW = 0x3c
    REG_ACCEL_Y_HI = 0x3d
    REG_ACCEL_Y_LOW = 0x3e
    REG_ACCEL_Y_HI = 0x3f
    REG_ACCEL_Y_LOW = 0x40

    REG_GYRO_X_HI = 0x43
    REG_GYRO_X_LOW = 0x44
    REG_GYRO_Y_HI = 0x45
    REG_GYRO_Y_LOW = 0x46
    REG_GYRO_Z_HI = 0x47
    REG_GYRO_Z_LOW = 0x48
    
    REG_MAG_X_HI = 0x03
    REG_MAG_X_LOW = 0x04
    REG_MAG_Y_HI = 0x07
    REG_MAG_Y_LOW = 0x08
    REG_MAG_Z_HI = 0x05
    REG_MAG_Z_LOW = 0x06
    # Not yet implemented 
    #removeGravity = False
    #lp_g = [0,0,0]

    def __init__(self):
        self.i2c_imu = Adafruit_I2C(self.MPU6050_I2C_ADDRESS)
        # Power reset, needed to wake up the MPU6050
        self.i2c_imu.write8(self.REG_IMU_RESET, 0)
        
        self.i2c_mag = Adafruit_I2C(self.HMC5883L_I2C_ADDRESS)
        # Set the Magnometer to continous read mode
        self.i2c_mag.write8(self.REG_MAG_CONT_MODE, 0)

    @staticmethod
    def _twos_comp(val, bits):
        if((val&(1<<(bits-1))) != 0 ):
            val = val - (1<<bits)
        return val

    def _reg2raw(self, i2c, high, low):
        b_high = i2c.readU8(high) & 0xFF
        b_low = i2c.readU8(low) & 0xFF
        return self._twos_comp((b_high << 8 | b_low), 16)

    # Obtain the gravity factor for the current plane of motion 
    def calibrate(self, duration=1):
        start = time.time()
        while (start - time.time() < duration):
            # Apply a low pass filter to obtain a gravity factor
            a = getRawAccel()
            self.g = [0.9*self.g[i] + 0.1*a[i]for i in range(3)]

    def getRawAccel(self):
        a = [self.getRawAccelX(), self.getRawAccelY(), self.getRawAccelZ()]
        
        # Update the current gravity factor 
        self.g = [0.9*self.g[i] + 0.1*a[i]for i in range(3)]
        # Run through low pass filter
        return a

    def getRawAccelX(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_ACCEL_X_HI, self.REG_ACCEL_X_LOW)
        return float(raw)/self.RAW2ACCEL 

    def getRawAccelY(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_ACCEL_Y_HI, self.REG_ACCEL_Y_LOW)
        return float(raw)/self.RAW2ACCEL 

    def getRawAccelZ(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_ACCEL_Z_HI, self.REG_ACCEL_Z_LOW)
        return float(raw)/self.RAW2ACCEL 

    def getRawGyroX(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_GYRO_X_HI, self.REG_GYRO_X_LOW)
        return float(raw)/self.RAW2GYRO

    def getRawGyroY(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_GYRO_Y_HI, self.REG_GYRO_Y_LOW)
        return float(raw)/self.RAW2GYRO

    def getRawGyroZ(self):
        raw = self._reg2raw(self.i2c_imu, self.REG_GYRO_Z_HI, self.REG_GYRO_Z_LOW)
        return float(raw)/self.RAW2GYRO

    def getRawMagX(self):
        raw = self._reg2raw(self.i2c_mag, self.REG_MAG_X_HI, self.REG_MAG_X_LOW)
        return float(raw)/self.RAW2MAG

    def getRawMagY(self):
        raw = self._reg2raw(self.i2c_mag, self.REG_MAG_Y_HI, self.REG_MAG_Y_LOW)
        return float(raw)/self.RAW2MAG

    def getRawMagZ(self):
        raw = self._reg2raw(self.i2c_mag, self.REG_MAG_Z_HI, self.REG_MAG_Z_LOW)
        return float(raw)/self.RAW2MAG


def rawIMU():
        raw_imu = IMU()
        pub_imu  = rospy.Publisher('imu/data_raw', sensor_msgs.msg.Imu, queue_size = 10)
        pub_magno = rospy.Publisher('imu/mag', Vector3, queue_size = 10)
        r = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            angular_vel = Vector3()
            linear_accel = Vector3()
            magnometer = Vector3()
            header = Header()

            angular_vel.x = IMU.getRawGyroX()
            angular_vel.y = IMU.getRawGyroY()
            angular_vel.z = IMU.getRawGyroZ()
            linear_accel.x = IMU.getRawAccelX()
            linear_accel.y = IMU.getRawAccelY()
            linear_accel.z = IMU.getRawAccelZ()
            magnometer.x = IMU.getRawMagX() 
            magnometer.y = IMU.getRawMagY()
            magnometer.z = IMU.getRawMagZ()

            # Fill header and for timestamps
            rostime = rospy.get_rostime()
            header.secs = rostime.secs
            header.nsecs = rostime.nsecs 
            header.frame_id = 1 # Declare to be the global frame

            raw_imu_data = sensor_msgs.msg.Imu()
            raw_imu_data.angular_velocity = angular_velocity
            raw_imu_data.linear_acceleration = linear_accel
            raw_imu_data.header = header

            pub_imu.publish(raw_imu_data)
            pub_magno.publish(magnometer)
            r.sleep()

if __name__ == '__main__':
    try:
        rawIMU()
    except rospy.ROSInterruptException: pass

