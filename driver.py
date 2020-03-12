import smbus
import time as t
import numpy as np
from madgwickahrs import MadgwickAHRS

I2C_IMU_ADDRESS = 0x69
I2C_ADDR = 0x68
I2C_ADDR_ALT = 0x69

ICM20948_I2C_MST_CTRL = 0x01
ICM20948_I2C_MST_DELAY_CTRL = 0x02
ICM20948_I2C_SLV0_ADDR = 0x03
ICM20948_I2C_SLV0_REG = 0x04
ICM20948_I2C_SLV0_CTRL = 0x05
ICM20948_I2C_SLV0_DO = 0x06
ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B


# Bank 0
ICM20948_WHO_AM_I = 0x00
ICM20948_USER_CTRL = 0x03
ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07
ICM20948_INT_PIN_CFG = 0x0F

ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10
ICM20948_ACCEL_SMPLRT_DIV_2 = 0x11
ICM20948_ACCEL_INTEL_CTRL = 0x12
ICM20948_ACCEL_WOM_THR = 0x13
ICM20948_ACCEL_CONFIG = 0x14
ICM20948_ACCEL_XOUT_H = 0x2D
ICM20948_GRYO_XOUT_H = 0x33

AK09916_I2C_ADDR = 0x0c
AK09916_CHIP_ID = 0x09
AK09916_WIA = 0x01
AK09916_ST1 = 0x10
AK09916_ST1_DOR = 0b00000010   # Data overflow bit
AK09916_ST1_DRDY = 0b00000001  # Data self.ready bit
AK09916_HXL = 0x11
AK09916_ST2 = 0x18
AK09916_ST2_HOFL = 0b00001000  # Magnetic sensor overflow bit
AK09916_CNTL2 = 0x31
AK09916_CNTL2_MODE = 0b00001111
AK09916_CNTL2_MODE_OFF = 0
AK09916_CNTL2_MODE_SINGLE = 1
AK09916_CNTL2_MODE_CONT1 = 2
AK09916_CNTL2_MODE_CONT2 = 4
AK09916_CNTL2_MODE_CONT3 = 6
AK09916_CNTL2_MODE_CONT4 = 8
AK09916_CNTL2_MODE_TEST = 16
AK09916_CNTL3 = 0x32


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
    #mag_write(AK09916_CNTL2, 0x01) #Set magnetometer to single measurement mode
    ready = mag_read(AK09916_ST1) & 0x01
    #while (ready != 1): # Wait until data is ready
        #time.sleep(0.00001)
    set_bank(3) # set bank to 3
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_CTRL, 0x80 | 0x06) # Enable storing data from mgnetometer and 1 byte to read
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80) # set slave address and read
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_REG, 0x11) # Read from register
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_DO, 0xff) # Set slave DO?
    set_bank(0)
    a = bus.read_i2c_block_data(I2C_IMU_ADDRESS, 0x3b,6) # 3b should be the first address to write to
    print (a)
    return a


def set_bank(bank):
    bank = (bank << 4)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x7f,bank)
def mag_write(reg,value):
    set_bank(3) # Set bank to 3
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR) # Set slave 0 address to i2c address of magnetometer
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_REG,reg) # Set register on magnetomer to write to
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_DO,value) # Output data to slave 0
    set_bank(0) # Switch back to bank 0
def mag_read(reg):    
    set_bank(3)
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_CTRL, 0x80 | 1)  # Enable storing data at EXT_SENS_DATA register and number of bytes to 1
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80) # set register of magnetomer and set to read
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_REG, reg) #Set register to read from
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_I2C_SLV0_DO, 0xff) # Read data
    set_bank(0)
    return read_data(ICM20948_EXT_SLV_SENS_DATA_00)
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
    read_mag_data()
    mag_x =1 #get_mag_decimal(0x3C, 0x3B)
    mag_y =1 #get_mag_decimal(0x3E, 0x3D)
    mag_z =1 #get_mag_decimal(0x40, 0x3D)
    # print("mag x: ", mag_x,"mag y: ",mag_y,"mag z: ",mag_z)

    return np.array([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z])

def main():
    success = False
    while not success:
        try:
            set_bank(1)
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_1, 0x01) # wake up imu from sleep, try until works 
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0x00) # Set accelerometer and gyroscope to on
            set_bank(3) # Change to bank 3
            bus.write_byte_data(I2C_IMU_ADDRESS,ICM20948_I2C_MST_CTRL,0x4d) # Set i2c control for magnetometer. Multimaster capability, and i2c clockspeed of 7, recommended
            bus.write_byte_data(I2C_IMU_ADDRESS,ICM20948_I2C_MST_DELAY_CTRL,0x01) # Set slave 0 to only be accessed every x samples
            mag_write(AK09916_CNTL3, 0x01) # soft reset the magnetometer
            success = True
            
        except:
            pass

    while (mag_read(AK09916_CNTL3) == 0x01): # Wait until magnetometer registers are initialized
        time.sleep(0.0001) #Basically never happens in the first place
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
        

if(__name__ == '__main__'):
    main()
