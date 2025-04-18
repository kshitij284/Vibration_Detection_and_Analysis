#include "mpu6050.h"
#include "kiss_fft.h"
#include <vector>
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

void MPU6050::collectAccelerationData(int rate)
{
    chrono::milliseconds sleep_time(1000 / rate);

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        readRawData(Ax, Ay, Az, Gx, Gy, Gz);
        accele_x[i] = Ax;
        accele_y[i] = Ay;
        accele_z[i] = Az;
        ThisThread::sleep_for(sleep_time);
    }
}

void MPU6050::printAccelerationData()
{
    printf("Acceleration Data:\n");
    printf("Ax: ");
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        printf(" %d ,", accele_x[i]);
    }
    printf("\n");
    printf("Ay: ");
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        printf(" %d ,", accele_y[i]);
    }
    printf("\n");
    printf("Az: ");
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        printf(" %d ,", accele_z[i]);
    }
    printf("\n");
}

void MPU6050::FFT_wAxis(char axis)
{
    kiss_fft_cfg cfg = kiss_fft_alloc(SAMPLE_SIZE, 0, NULL, NULL);
    int16_t *selectedArray = nullptr;

    // Select the appropriate array based on the axis input
    switch (axis)
    {
    case 'x':
        selectedArray = accele_x;
        break;
    case 'y':
        selectedArray = accele_y;
        break;
    case 'z':
        selectedArray = accele_z;
        break;
    default:
        printf("Invalid axis. Please use 'x', 'y', or 'z'.\n");
        kiss_fft_free(cfg);
        return;
    }

    // Prepare FFT input
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        fft_input[i].r = (float)selectedArray[i];
        fft_input[i].i = 0.0f;
    }

    // Perform FFT
    kiss_fft(cfg, fft_input, fft_output);

    // Calculate magnitude
    for (int i = 0; i < SAMPLE_SIZE / 2; i++)
    {
        fft_magnitude[i] = sqrt(fft_output[i].r * fft_output[i].r + fft_output[i].i * fft_output[i].i);
    }

    kiss_fft_free(cfg);
    cfg = NULL;
}

float *MPU6050::getFFTMagnitude()
{
    return fft_magnitude;
}


int MPU6050::getDominantFrequency(float samplingRate, char axis)
{

    // Perform FFT on the selected axis
    FFT_wAxis(axis);

    // Initialize variables to find the maximum frequency
    int maxIndex = 0;
    float maxValue = 0;

    // Find the frequency bin with maximum magnitude (skipping DC component at index 0)
    for (int i = 1; i < SAMPLE_SIZE / 2; i++)
    {
        if (fft_magnitude[i] > maxValue)
        {
            maxValue = fft_magnitude[i];
            maxIndex = i;
        }
    }

    // Convert bin index to frequency in Hz
    float dominantFreq = maxIndex * samplingRate / SAMPLE_SIZE;
    return round(dominantFreq);
}