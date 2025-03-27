#include "mpu6050.h"

MPU6050::MPU6050(PinName sda, PinName scl) : i2c(sda, scl) {}

void MPU6050::writeRegister(uint8_t reg, uint8_t data)
{
    char cmd[2] = {reg, data};
    i2c.write(MPU6050_ADDR, cmd, 2);
}

uint8_t MPU6050::readRegister(uint8_t reg)
{
    char data;
    i2c.write(MPU6050_ADDR, (char *)&reg, 1, true);
    i2c.read(MPU6050_ADDR, &data, 1);
    return data;
}

void MPU6050::init()
{
    writeRegister(0x6B, 0x00);
    writeRegister(0x1C, 0x00); // accelerometer : register 28 (binary)
    writeRegister(0x1B, 0x00); // gyro ; register 27 (binary)
}

bool MPU6050::testConnection()
{
    return readRegister(0x75) == 0x68;
}

void MPU6050::readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
    char data[14];
    char reg = 0x3B;
    i2c.write(MPU6050_ADDR, &reg, 1, true);
    i2c.read(MPU6050_ADDR, data, 14);

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];
    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];
}
