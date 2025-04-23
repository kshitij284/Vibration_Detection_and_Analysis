#ifndef MPU6050_H
#define MPU6050_H

#include "mbed.h"
#include "kiss_fft.h"

#define SAMPLE_SIZE 256
#define FFT_SIZE SAMPLE_SIZE
#define DEFAULT_SAMPLING_RATE 100 // Hz

class MPU6050
{
public:
    MPU6050(PinName sda, PinName scl);
    void init();
    void reset();
    bool testConnection();
    void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
    void collectAccelerationData(int rate = DEFAULT_SAMPLING_RATE);
    void printAccelerationData();
    void printAccelerationDatainG();
    void calibrateAccelerometer();

    void FFT_wAxis(char axis);
    void FFT_wFilter();
    float *getFFTMagnitude();
    int getDominantFrequency(float samplingRate = DEFAULT_SAMPLING_RATE, char axis = 'x');

private:
    I2C i2c;
    static constexpr uint8_t MPU6050_ADDR = 0x68 << 1; // 7bit address to 8bit
    int16_t Ax, Ay, Az, Gx, Gy, Gz;
    int16_t accele_x[SAMPLE_SIZE];
    int16_t accele_y[SAMPLE_SIZE];
    int16_t accele_z[SAMPLE_SIZE];
    float accele_bias[3] = {0.0f, 0.0f, 0.0f};  // Bias for x, y, z axes
    float accele_scale[3] = {1.0f, 1.0f, 1.0f}; // Scale for x, y, z axes

    kiss_fft_cpx fft_input[FFT_SIZE];
    kiss_fft_cpx fft_output[FFT_SIZE];
    float fft_magnitude[FFT_SIZE / 2];
    int top_three_freqeuncy[3];

    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif
