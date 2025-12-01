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
    Serial.println("    Starting I2C...");
    Wire.begin(sda, scl);
    Wire.setClock(400000); // 400kHz
    
    Serial.println("    Connecting to MPU6050...");
    if (!_mpu.begin()) {
        Serial.println("    ERROR: MPU6050 not found!");
        return false;
    }
    
    Serial.println("    MPU6050 found!");
    Serial.println("    Configuring MPU6050...");
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    delay(100);
    calibrate();
    
    _lastTime = micros();
    return true;
}

void IMU::calibrate() {
    Serial.println("    Calibrating IMU (1000 samples)...");
    
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
        
        // 每100次采样打印一个点，避免看门狗超时
        if (i % 100 == 0) {
            Serial.print(".");
            yield(); // 让系统处理其他任务，喂狗
        }
        
        delay(2);
    }
    Serial.println(); // 换行
    
    _gyroOffsetX = sumGyroX / samples;
    _gyroOffsetY = sumGyroY / samples;
    _gyroOffsetZ = sumGyroZ / samples;
    _accelOffsetX = sumAccelX / samples;
    _accelOffsetY = sumAccelY / samples;
    _accelOffsetZ = sumAccelZ / samples - 9.81; // Gravity compensation
    
    Serial.println("    Calibration complete!");
    Serial.printf("    Gyro offsets: X=%.3f Y=%.3f Z=%.3f\n", _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
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
    // 注意：这里交换了pitch和roll的计算，因为传感器安装方向
    float accelPitch = atan2(-ax, az) * 180.0 / PI;  // 原来的roll公式
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;  // 原来的pitch公式
    
    // Complementary filter
    float alpha = 0.98;
    
    // 更新Pitch角（左右倾斜 - 平衡车使用）
    _pitch = alpha * (_pitch + gy * dt * 180.0 / PI) + (1.0 - alpha) * accelPitch;
    
    // 更新Roll角（前后倾斜）
    _roll = alpha * (_roll + gx * dt * 180.0 / PI) + (1.0 - alpha) * accelRoll;
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