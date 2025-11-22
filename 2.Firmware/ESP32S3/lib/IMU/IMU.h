#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU {
public:
    IMU();
    
    bool init(uint8_t sda, uint8_t scl);
    void update();
    
    float getPitch();
    float getRoll();
    float getGyroZ();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    
    void calibrate();
    
private:
    Adafruit_MPU6050 _mpu;
    
    float _pitch;
    float _roll;
    float _gyroZ;
    float _accelX, _accelY, _accelZ;
    
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;
    float _accelOffsetX, _accelOffsetY, _accelOffsetZ;
    
    unsigned long _lastTime;
    
    void complementaryFilter(float accelAngle, float gyroRate, float &angle, float alpha);
};

#endif