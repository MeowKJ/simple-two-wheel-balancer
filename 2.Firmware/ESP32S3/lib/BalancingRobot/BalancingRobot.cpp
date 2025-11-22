#include "BalancingRobot.h"
#include "board.h"

BalancingRobot::BalancingRobot() {
    _targetAngle = 0;
    _targetSpeed = 0;
    _targetTurn = 0;
    _angleOutput = 0;
    _speedOutput = 0;
    _turnOutput = 0;
    _isBalancing = false;
}

void BalancingRobot::init() {
    // Initialize motors
    _leftMotor = new MotorDriver(LMOTOR_IN1_PIN, LMOTOR_IN2_PIN, 
                                  LMOTOR_SLEEP_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0);
    _rightMotor = new MotorDriver(RMOTOR_IN1_PIN, RMOTOR_IN2_PIN, 
                                   RMOTOR_SLEEP_PIN, MCPWM_UNIT_0, MCPWM_TIMER_1);
    
    _leftMotor->init();
    _rightMotor->init();
    
    // Initialize encoders
    _leftEncoder = new Encoder(LMOTOR_ENCODER_A_PIN, LMOTOR_ENCODER_B_PIN, PCNT_UNIT_0);
    _rightEncoder = new Encoder(RMOTOR_ENCODER_A_PIN, RMOTOR_ENCODER_B_PIN, PCNT_UNIT_1);
    
    _leftEncoder->init();
    _rightEncoder->init();
    
    // Initialize IMU
    _imu = new IMU();
    if (!_imu->init(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("Failed to initialize IMU!");
        while (1) delay(10);
    }
    
    // Initialize PID controllers
    // Angle PID: controls balance angle
    _anglePID = new PIDController(40.0, 0.0, 1.5);
    _anglePID->setOutputLimits(-100, 100);
    
    // Speed PID: controls forward/backward movement
    _speedPID = new PIDController(0.5, 0.1, 0.0);
    _speedPID->setOutputLimits(-10, 10);
    
    // Turn PID: controls rotation
    _turnPID = new PIDController(2.0, 0.0, 0.1);
    _turnPID->setOutputLimits(-30, 30);
    
    Serial.println("Balancing robot initialized!");
}

void BalancingRobot::update() {
    // Update IMU
    _imu->update();
    float pitch = _imu->getPitch();
    
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
    
    // Calculate average speed
    float avgSpeed = (_leftEncoder->getSpeed() + _rightEncoder->getSpeed()) / 2.0;
    
    // Speed control (adjust target angle based on speed error)
    _speedOutput = _speedPID->compute(_targetSpeed, avgSpeed);
    float angleSetpoint = _targetAngle + _speedOutput;
    angleSetpoint = constrain(angleSetpoint, -15, 15);
    
    // Angle control (main balance control)
    _angleOutput = _anglePID->compute(angleSetpoint, pitch);
    
    // Turn control
    float speedDiff = _leftEncoder->getSpeed() - _rightEncoder->getSpeed();
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
    return _imu->getPitch();
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