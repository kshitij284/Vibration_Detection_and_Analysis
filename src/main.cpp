#include "mbed.h"

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

// MPU6050 I2C Address
#define MPU_ADDR 0x68 << 1 // 7-bit address shifted to 8-bit

// WHO_AM_I register
#define WHO_AM_I_REG 0x75

I2C i2c(SDA_PIN, SCL_PIN);

int main()
{
    char received[1];
    uint8_t reg_whoAmI = WHO_AM_I_REG;
    i2c.write(MPU_ADDR, (char *)&reg_whoAmI, 1, true);

    i2c.read(MPU_ADDR, received, 1);

    printf("WHO_AM_I value: 0x%X\n", received[0]);
    while(1){
        ThisThread::sleep_for(1000ms);
        printf("WHO_AM_I value: 0x%X\n", received[0]);
    }
}
