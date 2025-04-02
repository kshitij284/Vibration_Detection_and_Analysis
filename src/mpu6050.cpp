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
    writeRegister(0x1C, 0x00); // accelerometer : register 28 (binary)
    writeRegister(0x1B, 0x00); // gyro ; register 27 (binary)
}

void MPU6050::reset()
{
    char config[2];

    // Write to Power Management Register (0x6B) - Set Reset Bit (Bit 7)
    config[0] = 0x6B;
    config[1] = 0x80; // Reset bit set
    i2c.write(MPU6050_ADDR, config, 2);

    // Wait for reset to complete
    ThisThread::sleep_for(100ms);

    printf("MPU6050 Reset Done\n");
}

bool MPU6050::testConnection()
{
    return readRegister(0x75) == 0x68;
}

void MPU6050::readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
    char data[14];
    char reg = 0x3B;
    i2c.write(MPU6050_ADDR, &reg, 1, true);
    i2c.read(MPU6050_ADDR, data, 14);

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];
    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];
}

void MPU6050::collectAccelerationData()
{
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        readRawData(Ax, Ay, Az, Gx, Gy, Gz);
        accele_x[i] = Ax;
        ThisThread::sleep_for(10ms);
    }
}

void MPU6050::FFT()
{
    for (int i = 0; i < FFT_SIZE; i++)
    {
        input_f32[i] = 0.0f;
    }

    for (int i = 0; i < FFT_SIZE; i++)
    {
        printf("Accele_x[%d]: %d\n", i, accele_x[i]);

        input_f32[i] = (float32_t)accele_x[i];

        printf("input_f32[%d]: %f\n", i, input_f32[i]);
    }

    arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS)
    {
        printf("FFT initialization failed!\n");
    }

    arm_rfft_fast_f32(&fft_instance, input_f32, output_f32, 0);

    printf("FFT Analysis:\n");
    for (int i = 0; i < FFT_SIZE / 2; i++)
    {
        printf("Freq Bin %d: %f\n", i, output_f32[i]);
    }
}