#include "imu.h"

void setup() {
    // delay(5000);

    // Initialize serial communication
    Serial.begin(115200);

    // Calibrate IMU
    IMU::instance().calibrate();
}

void loop() {
    IMU& imu = IMU::instance();

    // Get IMU informations
    position_t pos = imu.getPosition();
    rotation_t rot = imu.getRotation();

    delay(1000);
}