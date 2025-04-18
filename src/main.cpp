#include "mbed.h"
#include "mpu6050.h"
#include "kiss_fft.h"
#include <cstdio>

#define SDA_PIN PB_9 // Pins for I2C communication
#define SCL_PIN PB_8

#define SAMPLING_RATE 300

MPU6050 mpu(SDA_PIN, SCL_PIN);
PwmOut servo(PA_6);

Thread servo_thread;
Thread mpu_thread;

void set_servo_angle(float angle_deg)
{
    // Clamp angle between 0 and 180
    if (angle_deg < 0)
        angle_deg = 0;
    if (angle_deg > 180)
        angle_deg = 180;

    // Convert angle to pulse width (in seconds)
    float min_pulse = 0.001; // 1 ms
    float max_pulse = 0.002; // 2 ms
    float pulse_width = min_pulse + (angle_deg / 180.0f) * (max_pulse - min_pulse);

    servo.pulsewidth(pulse_width);
}

void servo_task()
{
    servo.period(0.02); // 20 ms period (50 Hz)

    while (true)
    {
        for (float angle = 0; angle <= 180; angle += 10)
        {
            set_servo_angle(angle);
            ThisThread::sleep_for(50ms);
        }

        for (float angle = 180; angle >= 0; angle -= 10)
        {
            set_servo_angle(angle);
            ThisThread::sleep_for(50ms);
        }
    }
}

void mpu_task()
{
    while (true)
    {
        mpu.collectAccelerationData(SAMPLING_RATE);
        mpu.printAccelerationData();
        int dominantFreq = mpu.getDominantFrequency(SAMPLING_RATE, 'x');
        printf("Dominant Frequency with kissfft for x axis: %d Hz\n", dominantFreq);

        int dominantFreq = mpu.getDominantFrequency(SAMPLING_RATE, 'y');
        printf("Dominant Frequency with kissfft for y axis: %d Hz\n", dominantFreq);

        int dominantFreq = mpu.getDominantFrequency(SAMPLING_RATE, 'z');
        printf("Dominant Frequency with kissfft for z axis: %d Hz\n", dominantFreq);
        
        ThisThread::sleep_for(1000ms); // Sleep for 1 second
    }
}

int main()
{
    mpu.init();
    if (!mpu.testConnection())
    {
        printf("MPU6050 connection failed!\n");
        return -1;
    }
    printf("Test:  with no external vibrations\n");

    while (1)
    {
        mpu_thread.start(mpu_task);
        servo_thread.start(servo_task);
        ThisThread::sleep_for(1000ms);
    }
}