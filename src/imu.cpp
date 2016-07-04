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

    ax0 /= calibrateCalls;
    ay0 /= calibrateCalls;
    az0 /= calibrateCalls;

    gx0 /= calibrateCalls;
    gy0 /= calibrateCalls;
    gz0 /= calibrateCalls;


    mMpu6050.setXAccelOffset(- ax0 / 8);
    mMpu6050.setYAccelOffset(- ay0 / 8);
    mMpu6050.setZAccelOffset((accelerationToQuid(1.0) - az0) / 8);

    mMpu6050.setXGyroOffset(- gx0 / 4);
    mMpu6050.setYGyroOffset(- gy0 / 4);
    mMpu6050.setZGyroOffset(- gz0 / 4);

    Serial.println(" done");

    // Serial.print(quidToAcceleration(ax0)); Serial.print('\t');
    // Serial.print(quidToAcceleration(ay0)); Serial.print('\t');
    // Serial.print(quidToAcceleration(az0)); Serial.print('\t');

    // Serial.print(quidToDegree(gx0)); Serial.print('\t');
    // Serial.print(quidToDegree(gy0)); Serial.print('\t');
    // Serial.print(quidToDegree(gz0)); Serial.println();

    // Serial.println();
}

int16_t IMU::getAccelerationRange()
{
    switch(mMpu6050.getFullScaleAccelRange())
    {
        case MPU6050_ACCEL_FS_2:
            return 2;
        case MPU6050_ACCEL_FS_4:
            return 4;
        case MPU6050_ACCEL_FS_8:
            return 8;
        case MPU6050_ACCEL_FS_16:
            return 16;
    }

    return 0;
}

int16_t IMU::getGyroscopeRange()
{
    switch(mMpu6050.getFullScaleGyroRange())
    {
        case MPU6050_GYRO_FS_250:
            return 250;
        case MPU6050_GYRO_FS_500:
            return 500;
        case MPU6050_GYRO_FS_1000:
            return 1000;
        case MPU6050_GYRO_FS_2000:
            return 2000;
    }

    return 0;
}

double IMU::quidToAcceleration(int16_t quid)
{
    const double accelScale = static_cast<double>(SHRT_MAX) / getAccelerationRange();

    return static_cast<double>(quid) / accelScale;
}

double IMU::quidToDegree(int16_t quid)
{
    const double gyroScale = static_cast<double>(SHRT_MAX) / getGyroscopeRange();

    return static_cast<double>(quid) / gyroScale;    
}

int16_t IMU::accelerationToQuid(double acceleration)
{
    const double accelScale = static_cast<double>(SHRT_MAX) / getAccelerationRange();

    return static_cast<int16_t>(acceleration * accelScale);
}

int16_t IMU::degreeToQuid(double degree)
{
    const double gyroScale = static_cast<double>(SHRT_MAX) / getGyroscopeRange();

    return static_cast<int16_t>(degree * gyroScale);
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

    Serial.print(quidToAcceleration(ax)); Serial.print('\t');
    Serial.print(quidToAcceleration(ay)); Serial.print('\t');
    Serial.print(quidToAcceleration(az)); Serial.print('\t');

    Serial.print(quidToDegree(gx)); Serial.print('\t');
    Serial.print(quidToDegree(gy)); Serial.print('\t');
    Serial.print(quidToDegree(gz)); Serial.println();

    return position_t();
}

const rotation_t IMU::getRotation()
{
    return rotation_t();
}
