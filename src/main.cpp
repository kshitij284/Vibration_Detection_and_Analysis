#include "mbed.h"
#include "mpu6050.h"

#include "arm_math.h"

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

MPU6050 mpu(SDA_PIN, SCL_PIN);

int main()
{
    mpu.init();

    while (1)
    {
        printf("Collectin vibration data\n");
        mpu.collectAccelerationData();

        printf("Freqency Analysis");
        mpu.FFT();

        ThisThread::sleep_for(1s);
    }
}
