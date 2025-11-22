#ifndef BALANCING_ROBOT_H
#define BALANCING_ROBOT_H

#include <Arduino.h>
#include "MotorDriver.h"
#include "Encoder.h"
#include "IMU.h"
#include "PIDController.h"

class BalancingRobot {
public:
    BalancingRobot();
    
    void init();
    void update();
    void setTargetAngle(float angle);
    void setTargetSpeed(float speed);
    void setTargetTurn(float turn);
    void stop();
    void emergencyStop();
    
    float getPitch();
    float getLeftSpeed();
    float getRightSpeed();
    
    bool isBalancing();
    
    // PID tuning
    void setAnglePID(float kp, float ki, float kd);
    void setSpeedPID(float kp, float ki, float kd);
    void setTurnPID(float kp, float ki, float kd);
    
private:
    MotorDriver *_leftMotor;
    MotorDriver *_rightMotor;
    Encoder *_leftEncoder;
    Encoder *_rightEncoder;
    IMU *_imu;
    
    PIDController *_anglePID;
    PIDController *_speedPID;
    PIDController *_turnPID;
    
    float _targetAngle;
    float _targetSpeed;
    float _targetTurn;
    
    float _angleOutput;
    float _speedOutput;
    float _turnOutput;
    
    bool _isBalancing;
    
    static constexpr float BALANCE_ANGLE_MIN = -45.0;
    static constexpr float BALANCE_ANGLE_MAX = 45.0;
};

#endif