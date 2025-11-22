#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit) {
    _pinA = pinA;
    _pinB = pinB;
    _unit = unit;
    _lastCount = 0;
    _lastTime = 0;
    _speed = 0;
}

void Encoder::init() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = _pinA,
        .ctrl_gpio_num = _pinB,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = _unit,
        .channel = PCNT_CHANNEL_0,
    };
    
    pcnt_unit_config(&pcnt_config);
    
    pcnt_config.pulse_gpio_num = _pinB;
    pcnt_config.ctrl_gpio_num = _pinA;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    
    pcnt_unit_config(&pcnt_config);
    
    pcnt_set_filter_value(_unit, 100);
    pcnt_filter_enable(_unit);
    
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);
    
    _lastTime = millis();
}

int16_t Encoder::getCount() {
    int16_t count;
    pcnt_get_counter_value(_unit, &count);
    return count;
}

void Encoder::reset() {
    pcnt_counter_clear(_unit);
    _lastCount = 0;
}

void Encoder::update() {
    unsigned long now = millis();
    unsigned long dt = now - _lastTime;
    
    if (dt >= 50) { // Update every 50ms
        int16_t count = getCount();
        int16_t delta = count - _lastCount;
        
        // Calculate RPM
        _speed = (delta * 60000.0) / (TOTAL_PULSES * dt);
        
        _lastCount = count;
        _lastTime = now;
    }
}

float Encoder::getSpeed() {
    return _speed;
}