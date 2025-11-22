#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "driver/mcpwm.h"

class MotorDriver {
public:
    MotorDriver(uint8_t in1, uint8_t in2, uint8_t sleep_pin, 
                mcpwm_unit_t unit, mcpwm_timer_t timer);
    
    void init();
    void setSpeed(int16_t speed); // -100 to 100
    void sleep();
    void wakeup();
    void brake();

private:
    uint8_t _in1;
    uint8_t _in2;
    uint8_t _sleep;
    mcpwm_unit_t _unit;
    mcpwm_timer_t _timer;
    
    void forward(uint8_t duty);
    void backward(uint8_t duty);
};

#endif