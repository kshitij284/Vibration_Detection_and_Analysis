#include "mbed.h"
#include "mpu6050.h"

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

MPU6050 mpu(SDA_PIN, SCL_PIN);

int main()
{
    mpu.init();
    // testing  driver

    if (mpu.testConnection())
    {
        printf("MPU connected successfully\n");
    }
    else
    {
        printf("MPU: connection error - not detected");
    }

    int16_t ax, ay, az, gx, gy, gz;

    while (1)
    {
        mpu.readRawData(ax, ay, az, gx, gy, gz);
        printf("Acc: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d \r\n", ax, ay, az, gx, gy, gz);
        ThisThread::sleep_for(100ms);
    }
}
