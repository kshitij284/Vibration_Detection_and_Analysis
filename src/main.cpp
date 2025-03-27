#include "mbed.h"
#include "mpu6050.h"

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

MPU6050 mpu(SDA_PIN, SCL_PIN);

int main()
{
    mpu.init();
    while (1)
    {
        // testing  driver

        if (mpu.testConnection())
        {
            printf("MPU connected successfully\n");
        }
        else
        {
            printf("MPU: connection error - not detected");
        }
        ThisThread::sleep_for(1000ms);
    }
}
