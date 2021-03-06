#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

#include <MPU6050.h>

#include "config.h"
#include "singleton.h"

struct position_t
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

struct rotation_t
{
    const uint16_t yaw;
    const uint16_t pitch;
    const uint16_t roll;    
};

class IMU : public Singleton<IMU>
{
public:
    void calibrate(unsigned long calibrateTimeMs = 10 * 1000);

    const position_t getPosition();
    const rotation_t getRotation();

    int16_t getAccelerationRange();
    int16_t getGyroscopeRange();

    double quidToAcceleration(int16_t quid);
    double quidToDegree(int16_t quid);

    int16_t accelerationToQuid(double acceleration);
    int16_t degreeToQuid(double degree);


private:
    IMU();

    static double sensitivity();

    MPU6050 mMpu6050;

    bool mConnectionOk;

    friend class Singleton<IMU>;
};

#endif /* IMU_H_INCLUDED */