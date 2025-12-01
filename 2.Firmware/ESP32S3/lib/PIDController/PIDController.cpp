#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0;
    _lastError = 0;
    _outMin = -100;
    _outMax = 100;
    _lastTime = 0;
}

float PIDController::compute(float setpoint, float input) {
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    
    if (_lastTime == 0 || dt > 1.0) {
        dt = 0.01; // Default dt for first run or if too much time passed
        _lastTime = now;
        _lastError = setpoint - input; // 初始化lastError
        return 0; // 第一次调用返回0，避免异常值
    }
    
    _lastTime = now;
    
    float error = setpoint - input;
    
    // Proportional
    float pTerm = _kp * error;
    
    // Integral with anti-windup
    if (abs(_ki) > 0.0001) { // 只有Ki足够大时才累加积分
        _integral += error * dt;
        // 根据Ki动态调整积分限幅
        float integralLimit = (_outMax - _outMin) / (2.0 * abs(_ki));
        _integral = constrain(_integral, -integralLimit, integralLimit);
    } else {
        _integral = 0; // Ki太小时清零积分
    }
    float iTerm = _ki * _integral;
    
    // Derivative
    float dTerm = 0;
    if (dt > 0.001) { // 确保dt不会太小导致除零
        dTerm = _kd * (error - _lastError) / dt;
    }
    
    _lastError = error;
    
    // Calculate output
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, _outMin, _outMax);
    
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _outMin = min;
    _outMax = max;
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
    _lastTime = 0;
}
