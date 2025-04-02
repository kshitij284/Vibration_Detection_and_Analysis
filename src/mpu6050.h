#ifndef MPU6050_H
#define MPU6050_H

#include "mbed.h"
#include "arm_math.h"

#define SAMPLE_SIZE 128
#define FFT_SIZE 128

class MPU6050
{
public:
    MPU6050(PinName sda, PinName scl);
    void init();
    void reset();
    bool testConnection();
    void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
    void collectAccelerationData();
    void FFT();

private:
    I2C i2c;
    arm_rfft_fast_instance_f32 fft_instance;
    static constexpr uint8_t MPU6050_ADDR = 0x68 << 1; // 7bit address to 8bit
    int16_t Ax, Ay, Az, Gx, Gy, Gz;
    int16_t accele_x[SAMPLE_SIZE];
    __attribute__((aligned(4))) float32_t input_f32[FFT_SIZE];
    __attribute__((aligned(4))) float32_t output_f32[FFT_SIZE * 2];

    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif
