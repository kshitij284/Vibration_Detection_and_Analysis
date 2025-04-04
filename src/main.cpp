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
        // Collect accelerometer data
        mpu.collectAccelerationData(SAMPLING_RATE);

        // Perform FFT on the collected data
        mpu.FFT();

        // // Get dominant frequency
        int domFreq = mpu.getDominantFrequency((float)SAMPLING_RATE);
        printf("Dominant frequency: %d Hz\n", domFreq);


        // Optional: Print the first few FFT magnitude values
        float *magnitudes = mpu.getFFTMagnitude();
        printf("FFT Magnitudes: ");
        for (int i = 0; i < 10; i++)
        {
            printf("%.2f ", magnitudes[i]);
        }
        printf("\n");

        ThisThread::sleep_for(1s);
    }
}