import smbus
import time as t
import numpy as np
from madgwickahrs import MadgwickAHRS

I2C_IMU_ADDRESS = 0x69
bus = smbus.SMBus(2)

filter = MadgwickAHRS()

def get_decimal(ls, ms):
    high = read_data(ms) << 8
    low = read_data(ls) & 0xff
    return np.int16((high | ls))
def get_mag_decimal(ms,ls):
    high = read_mag_data(ms) << 8 # little endian so ls is right 8 bits
    low = read_mag_data(ls) & 0xff
    return np.int16((high | low))

def read_data(num):
    a = bus.read_byte_data(I2C_IMU_ADDRESS, num)
    # print(a)
    return a

def read_mag_data():
    set_bank(3) # set bank to 3
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x05, 0x80 | 6) # set slave contorl Enable read data an bytes read
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x03, 0x0C | 0x80) # set slave address
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x04, 0x11) # Set slave register to 
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x06, 0xff) # Set slave DO?
    set_bank(0)
    a = bus.read_i2c_block_data(I2C_IMU_ADDRESS, 0x3b,6)
    print (a)
    return a


def set_bank(bank):
    bank = (bank << 4)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x7f,bank)
def mag_write(reg,value):
    set_bank(3)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x03,0x0c)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x04,reg)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x06,value)
    set_bank(0)
def get_data():
    elapsed_time = t.process_time()

    accel_x = get_decimal(0x2E, 0x2D)
    accel_y = get_decimal(0x30, 0x2F)
    accel_z = get_decimal(0x32, 0x31)
    # print("accel x: ", accel_x,"accel y: ",accel_y,"accel z: ",accel_z)
    gyro_x = get_decimal(0x34, 0x33)
    gyro_y = get_decimal(0x36, 0x35)
    gyro_z = get_decimal(0x38, 0x37)
    
    # all magnetometer data returns 0 because it is not being read correctly
    mag_write(0x31,0x01)
    read_mag_data()
    mag_x =1 # get_mag_decimal(0x3C, 0x3B)
    mag_y =1 # get_mag_decimal(0x3E, 0x3D)
    mag_z =1 # get_mag_decimal(0x40, 0x3D)
    # print("mag x: ", mag_x,"mag y: ",mag_y,"mag z: ",mag_z)

    return np.array([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z])

def main():
    success = False
    while not success:
        try:
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x06, 0x01) # wake up imu from sleep, try until works 
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x07, 0x00) # wake magnetometer from sleep, try until works
            set_bank(3)
            bus.write_byte_data(I2C_IMU_ADDRESS,0x01,0x4d)
            bus.write_byte_data(I2C_IMU_ADDRESS,0x02,0x01)
            mag_write(0x32, 0x01)
            success = True
        except:
            pass

    while(True):
        try:
            data = get_data()
        except:
            print("Connection Lost")
            t.sleep(1)
 
        # print("Accel: ", data[0], ",", data[1], ",", data[2])
        # print("Gyro: ", data[3], ",", data[4], ",", data[5])
        # print("Mag: ", data[6], ",", data[7], ",", data[8])
        # print()

        acc = np.array([data[0], data[1], data[2]])
        gyr = np.array([data[3], data[4], data[5]])
        mag = np.array([data[6], data[7], data[8]])
        gyr_rad = gyr * (np.pi/180)
    
        # filter.update(gyr_rad,acc,mag)
        # aboves update method can be run instead of update_imu if the magnetometer problem is fixed 
        filter.update_imu(gyr_rad,acc) #updates the filter and returns roll, pitch, and yaw in quaternion form
        ahrs = filter.quaternion.to_euler_angles()

        # values are between -pi and pi
        curRoll = ahrs[0]
        curPitch = ahrs[1]
        curYaw = ahrs[2]
        
        # print("Roll: ", curRoll, " Pitch: ", curPitch, " Yaw: ", curYaw)
        print()

if(__name__ == '__main__'):
    main()
