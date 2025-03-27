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
}

bool MPU6050::testConnection()
{
    return readRegister(0x75) == 0x68;
}
