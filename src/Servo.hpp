#ifndef SERBO_HPP
#define SERBO_HPP

#include "mbed.h"

class Servo
{
    public:
        Servo(const PinName& pin, const int max_deg, const std::chrono::microseconds max_pulse, const std::chrono::microseconds min_pulse, const int freq) : 
                    servo(pin), max_pulse_width(max), min_pulse_width(min), default_pulse_width((max_pulse_width + min_pulse_width) / 2), max_deg(max_deg)
        {
            servo.period_us(1'000'000 / freq);
            servo.pulsewidth_us(default_pulse_width.count());
        }
        move(const unsigned int deg)
        {
            const unsigned int actual_deg = std::clamp(deg, 0, max_deg);
            const int span = max_pulse_width.count() - min_pulse_width.count();
            const std::chrono::microseconds pulse_width = std::chrono::microseconds(span / max_deg * actual_deg) + min_pulse_width;
            servo.pulsewidth_us(pulse_width.count());
        }
    private:
        PwmOut servo;
        std::chrono::microseconds max_pulse_width;
        std::chrono::microseconds min_pulse_width;
        std::chrono::microseconds default_pulse_width;
        const int max_deg;
};