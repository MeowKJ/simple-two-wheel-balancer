#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h"

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit);
    
    void init();
    int16_t getCount();
    void reset();
    float getSpeed(); // RPM
    void update(); // Call periodically to calculate speed
    
    static const int PPR = 7;
    static const int GEAR_RATIO = 50;
    static const int TOTAL_PULSES = PPR * GEAR_RATIO * 4; // 1400 pulses per revolution

private:
    uint8_t _pinA;
    uint8_t _pinB;
    pcnt_unit_t _unit;
    int16_t _lastCount;
    unsigned long _lastTime;
    float _speed;
};

#endif