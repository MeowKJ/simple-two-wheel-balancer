#include "IMU.h"

IMU::IMU() {
    _pitch = 0;
    _roll = 0;
    _gyroZ = 0;
    _accelX = _accelY = _accelZ = 0;
    _gyroOffsetX = _gyroOffsetY = _gyroOffsetZ = 0;
    _accelOffsetX = _accelOffsetY = _accelOffsetZ = 0;
    _lastTime = 0;
}

bool IMU::init(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl);
    
    if (!_mpu.begin()) {
        return false;
    }
    
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    delay(100);
    calibrate();
    
    _lastTime = micros();
    return true;
}

void IMU::calibrate() {
    Serial.println("Calibrating IMU...");
    
    const int samples = 1000;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);
        
        sumGyroX += g.gyro.x;
        sumGyroY += g.gyro.y;
        sumGyroZ += g.gyro.z;
        sumAccelX += a.acceleration.x;
        sumAccelY += a.acceleration.y;
        sumAccelZ += a.acceleration.z;
        
        delay(2);
    }
    
    _gyroOffsetX = sumGyroX / samples;
    _gyroOffsetY = sumGyroY / samples;
    _gyroOffsetZ = sumGyroZ / samples;
    _accelOffsetX = sumAccelX / samples;
    _accelOffsetY = sumAccelY / samples;
    _accelOffsetZ = sumAccelZ / samples - 9.81; // Gravity compensation
    
    Serial.println("Calibration complete!");
}

void IMU::update() {
    sensors_event_t a, g, temp;
    _mpu.getEvent(&a, &g, &temp);
    
    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0;
    _lastTime = now;
    
    // Remove offsets
    float ax = a.acceleration.x - _accelOffsetX;
    float ay = a.acceleration.y - _accelOffsetY;
    float az = a.acceleration.z - _accelOffsetZ;
    float gx = g.gyro.x - _gyroOffsetX;
    float gy = g.gyro.y - _gyroOffsetY;
    float gz = g.gyro.z - _gyroOffsetZ;
    
    _accelX = ax;
    _accelY = ay;
    _accelZ = az;
    _gyroZ = gz;
    
    // Calculate pitch and roll from accelerometer
    float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float accelRoll = atan2(-ax, az) * 180.0 / PI;
    
    // Complementary filter
    float alpha = 0.98;
    complementaryFilter(accelPitch, gx * 180.0 / PI, _pitch, alpha);
    complementaryFilter(accelRoll, gy * 180.0 / PI, _roll, alpha);
}

void IMU::complementaryFilter(float accelAngle, float gyroRate, float &angle, float alpha) {
    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0;
    
    angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;
}

float IMU::getPitch() {
    return _pitch;
}

float IMU::getRoll() {
    return _roll;
}

float IMU::getGyroZ() {
    return _gyroZ;
}

float IMU::getAccelX() {
    return _accelX;
}

float IMU::getAccelY() {
    return _accelY;
}

float IMU::getAccelZ() {
    return _accelZ;
}