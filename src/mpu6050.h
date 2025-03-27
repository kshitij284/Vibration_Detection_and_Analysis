#ifndef MPU6050_H
#define MPU6050_H

#include "mbed.h"

class MPU6050
{
public:
    MPU6050(PinName sda, PinName scl);
    void init();
    bool testConnection();
    void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

private:
    I2C i2c;
    static constexpr uint8_t MPU6050_ADDR = 0x68 << 1; // 7bit address to 8bit
    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif
