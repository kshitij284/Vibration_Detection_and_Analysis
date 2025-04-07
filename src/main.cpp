#include "mbed.h"
#include "mpu6050.h"
#include "kiss_fft.h"
#include <cstdio>

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

#define SAMPLING_RATE 1000

MPU6050 mpu(SDA_PIN, SCL_PIN);

int main()
{
    mpu.init();
    if (!mpu.testConnection())
    {
        printf("MPU6050 connection failed!\n");
        return -1;
    }

    while (1)
    {

        mpu.repeatedFrequency(SAMPLING_RATE, 5);
        ThisThread::sleep_for(5000ms);
    }
}