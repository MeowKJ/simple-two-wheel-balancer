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
    }
    
    _lastTime = now;
    
    float error = setpoint - input;
    
    // Proportional
    float pTerm = _kp * error;
    
    // Integral with anti-windup
    _integral += error * dt;
    _integral = constrain(_integral, _outMin / _ki, _outMax / _ki);
    float iTerm = _ki * _integral;
    
    // Derivative
    float dTerm = 0;
    if (dt > 0) {
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