#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    
    float compute(float setpoint, float input);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();
    
    float getKp() { return _kp; }
    float getKi() { return _ki; }
    float getKd() { return _kd; }
    
private:
    float _kp, _ki, _kd;
    float _integral;
    float _lastError;
    float _outMin, _outMax;
    unsigned long _lastTime;
    float _lastDTerm;
    const float _dFilterTau = 0.1; // 微分滤波器常数
};

#endif