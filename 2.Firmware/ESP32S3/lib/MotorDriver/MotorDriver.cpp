#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t in1, uint8_t in2, uint8_t sleep_pin,
                         mcpwm_unit_t unit, mcpwm_timer_t timer) {
    _in1 = in1;
    _in2 = in2;
    _sleep = sleep_pin;
    _unit = unit;
    _timer = timer;
}

void MotorDriver::init() {
    pinMode(_sleep, OUTPUT);
    digitalWrite(_sleep, HIGH);
    
    mcpwm_gpio_init(_unit, 
                    (_timer == MCPWM_TIMER_0) ? MCPWM0A : MCPWM1A, 
                    _in1);
    mcpwm_gpio_init(_unit, 
                    (_timer == MCPWM_TIMER_0) ? MCPWM0B : MCPWM1B, 
                    _in2);
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(_unit, _timer, &pwm_config);
}

void MotorDriver::setSpeed(int16_t speed) {
    speed = constrain(speed, -100, 100);
    
    if (speed > 0) {
        forward(abs(speed));
    } else if (speed < 0) {
        backward(abs(speed));
    } else {
        brake();
    }
}

void MotorDriver::forward(uint8_t duty) {
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_B);
    mcpwm_set_duty(_unit, _timer, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(_unit, _timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void MotorDriver::backward(uint8_t duty) {
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_A);
    mcpwm_set_duty(_unit, _timer, MCPWM_OPR_B, duty);
    mcpwm_set_duty_type(_unit, _timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void MotorDriver::brake() {
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_A);
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_B);
}

void MotorDriver::sleep() {
    digitalWrite(_sleep, LOW);
}

void MotorDriver::wakeup() {
    digitalWrite(_sleep, HIGH);
}