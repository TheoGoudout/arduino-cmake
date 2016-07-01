#include "imu.h"

#include <limits.h>

#include <MPU6050.h>

IMU::IMU()
:
    mConnectionOk(false)
{
    Serial.print("Initializing I2C devices...");
    mMpu6050.initialize();
    Serial.println(" done");

    Serial.print("Testing device connections...");
    mConnectionOk = mMpu6050.testConnection();
    Serial.println(mConnectionOk ? " done" : " failed");
}

void IMU::calibrate(unsigned long calibrateTimeMs)
{
    if (!mConnectionOk)
    {
        Serial.println("Connection failure");
        return;
    }

    while (millis() <= 10*1000)
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        int16_t gx;
        int16_t gy;
        int16_t gz;

        // read raw accel/gyro measurements from device
        mMpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }

    double calibrateCalls = 0.0;
    
    double ax0 = 0.0;
    double ay0 = 0.0;
    double az0 = 0.0;

    double gx0 = 0.0;
    double gy0 = 0.0;
    double gz0 = 0.0;

    Serial.print("Calibrating MPU6050 for ");
    Serial.print(calibrateTimeMs);
    Serial.print(" ms...");

    const unsigned long calibrateEnd = millis() + calibrateTimeMs;
    while (millis() <= calibrateEnd)
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        int16_t gx;
        int16_t gy;
        int16_t gz;

        // read raw accel/gyro measurements from device
        mMpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ++calibrateCalls;

        ax0 += ax;
        ay0 += ay;
        az0 += az;

        gx0 += gx;
        gy0 += gy;
        gz0 += gz;
    }

    mMpu6050.setXAccelOffset(ax0 / calibrateCalls);
    mMpu6050.setYAccelOffset(ay0 / calibrateCalls);
    mMpu6050.setZAccelOffset(az0 / calibrateCalls);

    mMpu6050.setXGyroOffset(gx0 / calibrateCalls);
    mMpu6050.setYGyroOffset(gy0 / calibrateCalls);
    mMpu6050.setZGyroOffset(gz0 / calibrateCalls);

    Serial.println(" done");

    Serial.print(ax0 / calibrateCalls); Serial.print('\t');
    Serial.print(ay0 / calibrateCalls); Serial.print('\t');
    Serial.print(az0 / calibrateCalls); Serial.print('\t');

    Serial.print(gx0 / calibrateCalls); Serial.print('\t');
    Serial.print(gy0 / calibrateCalls); Serial.print('\t');
    Serial.print(gz0 / calibrateCalls); Serial.println();

    Serial.println();
}

const position_t IMU::getPosition()
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    // read raw accel/gyro measurements from device
    mMpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const double accelScale = SHRT_MAX / (2 * (mMpu6050.getFullScaleAccelRange() + 1));
    Serial.print(ax / accelScale); Serial.print('\t');
    Serial.print(ay / accelScale); Serial.print('\t');
    Serial.print(az / accelScale); Serial.print('\t');

    const double gyroScale = SHRT_MAX / (250 * (mMpu6050.getFullScaleGyroRange() + 1));
    Serial.print(gx / gyroScale); Serial.print('\t');
    Serial.print(gy / gyroScale); Serial.print('\t');
    Serial.print(gz / gyroScale); Serial.println();

    return position_t();
}


const rotation_t IMU::getRotation()
{
    return rotation_t();
}

double IMU::sensitivity()
{

}