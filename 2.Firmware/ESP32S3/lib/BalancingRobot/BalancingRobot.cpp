#include "BalancingRobot.h"

BalancingRobot::BalancingRobot() {
    _targetAngle = 0;
    _targetSpeed = 0;
    _targetTurn = 0;
    _angleOutput = 0;
    _speedOutput = 0;
    _turnOutput = 0;
    _isBalancing = false;
    
    // 关键：初始化所有指针为nullptr
    _leftMotor = nullptr;
    _rightMotor = nullptr;
    _leftEncoder = nullptr;
    _rightEncoder = nullptr;
    _imu = nullptr;
    _anglePID = nullptr;
    _speedPID = nullptr;
    _turnPID = nullptr;
}

void BalancingRobot::init(MotorPins leftMotor, MotorPins rightMotor, IMUPins imu) {
    Serial.println("  [1/5] Initializing left motor...");
    // Initialize motors
    _leftMotor = new MotorDriver(leftMotor.in1, leftMotor.in2, 
                                  leftMotor.sleep, MCPWM_UNIT_0, MCPWM_TIMER_0);
    _leftMotor->init();
    Serial.println("  [1/5] Left motor OK");
    
    Serial.println("  [2/5] Initializing right motor...");
    _rightMotor = new MotorDriver(rightMotor.in1, rightMotor.in2, 
                                   rightMotor.sleep, MCPWM_UNIT_0, MCPWM_TIMER_1);
    _rightMotor->init();
    Serial.println("  [2/5] Right motor OK");
    
    Serial.println("  [3/5] Initializing left encoder...");
    // Initialize encoders
    _leftEncoder = new Encoder(leftMotor.encoderA, leftMotor.encoderB, PCNT_UNIT_0, leftMotor.encoderReverse);
    _leftEncoder->init();
    Serial.println("  [3/5] Left encoder OK");
    
    Serial.println("  [4/5] Initializing right encoder...");
    _rightEncoder = new Encoder(rightMotor.encoderA, rightMotor.encoderB, PCNT_UNIT_1, rightMotor.encoderReverse);
    _rightEncoder->init();
    Serial.println("  [4/5] Right encoder OK");
    
    Serial.println("  [5/5] Initializing IMU (this may take 2-3 seconds)...");
    // Initialize IMU
    _imu = new IMU();
    if (!_imu->init(imu.sda, imu.scl)) {
        Serial.println("  [5/5] Failed to initialize IMU!");
        Serial.println("  Please check I2C connections!");
        Serial.println("  System will continue without IMU...");
        // 不要死循环，让系统继续运行
        delay(3000);
        return; // 或者可以设置一个错误标志
    }
    Serial.println("  [5/5] IMU OK");
    
    // Initialize PID controllers
    // Angle PID: controls balance angle
    _anglePID = new PIDController(5.0, 0.0, 0.5);
    _anglePID->setOutputLimits(-100, 100);
    
    // Speed PID: controls forward/backward movement
    _speedPID = new PIDController(2, 0.01, 0.0);
    _speedPID->setOutputLimits(-50, 50);
    
    // Turn PID: controls rotation
    _turnPID = new PIDController(0, 0.0, 0);
    _turnPID->setOutputLimits(-30, 30);
    
    Serial.println("Balancing robot initialized!");
}

void BalancingRobot::update() {
    // Update IMU
    _imu->update();
    float pitch = _imu->getPitch(); // 使用Pitch角（前后倾斜）

    // Update encoders
    _leftEncoder->update();
    _rightEncoder->update();

    // Check if robot is in balanceable range
    if (pitch < BALANCE_ANGLE_MIN || pitch > BALANCE_ANGLE_MAX) {
        emergencyStop();
        _isBalancing = false;
        return;
    }

    _isBalancing = true;

    // 归一化速度：将RPM转换为百分比（0-100）
    const float MAX_SPEED_RPM = 300.0;  // 电机最大转速
    float leftSpeedRaw = _leftEncoder->getSpeed();
    float rightSpeedRaw = _rightEncoder->getSpeed();

    // 转换为百分比
    float leftSpeedPercent = (leftSpeedRaw / MAX_SPEED_RPM) * 100.0;
    float rightSpeedPercent = (rightSpeedRaw / MAX_SPEED_RPM) * 100.0;
    float avgSpeedPercent = (leftSpeedPercent + rightSpeedPercent) / 2.0;

    // **角度控制 (main balance control)**
    // 角度控制是首先进行的，目的是让小车保持平衡
    _angleOutput = _anglePID->compute(_targetAngle, pitch);

    // **速度控制 (adjust target angle based on speed error)**
    // 在角度控制输出之后，调整目标角度来处理速度误差
    _speedOutput = _speedPID->compute(_targetSpeed, avgSpeedPercent);
    float angleSetpoint = _targetAngle + _speedOutput;
    angleSetpoint = constrain(angleSetpoint, -15, 15);  // 确保角度变化不会过大

    // **转向控制 (control rotation)**
    // 转向控制使用左右速度差来进行
    float speedDiff = leftSpeedPercent - rightSpeedPercent;
    _turnOutput = _turnPID->compute(_targetTurn, speedDiff);

    // Calculate motor outputs
    int16_t leftSpeed = _angleOutput - _turnOutput;
    int16_t rightSpeed = _angleOutput + _turnOutput;

    // Apply to motors
    _leftMotor->setSpeed(leftSpeed);
    _rightMotor->setSpeed(rightSpeed);
}


void BalancingRobot::setTargetAngle(float angle) {
    _targetAngle = constrain(angle, -10, 10);
}

void BalancingRobot::setTargetSpeed(float speed) {
    _targetSpeed = constrain(speed, -50, 50);
}

void BalancingRobot::setTargetTurn(float turn) {
    _targetTurn = constrain(turn, -50, 50);
}

void BalancingRobot::stop() {
    _leftMotor->brake();
    _rightMotor->brake();
    _targetSpeed = 0;
    _targetTurn = 0;
}

void BalancingRobot::emergencyStop() {
    _leftMotor->brake();
    _rightMotor->brake();
    _anglePID->reset();
    _speedPID->reset();
    _turnPID->reset();
}

float BalancingRobot::getPitch() {
    return _imu->getPitch(); // 返回Pitch角
}

float BalancingRobot::getRoll() {
    return _imu->getRoll(); // 返回Roll角
}

float BalancingRobot::getLeftSpeed() {
    return _leftEncoder->getSpeed();
}

float BalancingRobot::getRightSpeed() {
    return _rightEncoder->getSpeed();
}

bool BalancingRobot::isBalancing() {
    return _isBalancing;
}

void BalancingRobot::setAnglePID(float kp, float ki, float kd) {
    _anglePID->setTunings(kp, ki, kd);
}

void BalancingRobot::setSpeedPID(float kp, float ki, float kd) {
    _speedPID->setTunings(kp, ki, kd);
}

void BalancingRobot::setTurnPID(float kp, float ki, float kd) {
    _turnPID->setTunings(kp, ki, kd);
}