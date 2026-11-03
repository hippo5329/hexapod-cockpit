#include "hexa_servo.h"

#include <cmath>

namespace hexa {

ServoCal default_cal(int joint) {
    ServoCal c;
    if (joint % kJointsPerLeg == 2) c.zero_rad = -1.57079633f;
    return c;
}

int angle_to_us(const ServoCal& c, float rad, bool* clamped) {
    const float us = c.centre_us + c.trim_us + c.dir * (rad - c.zero_rad) * kUsPerRad;
    int v = static_cast<int>(std::lround(us));
    bool cl = false;
    if (v < c.min_us) { v = c.min_us; cl = true; }
    if (v > c.max_us) { v = c.max_us; cl = true; }
    if (clamped) *clamped = cl;
    return v;
}

float us_to_angle(const ServoCal& c, int us) {
    return c.zero_rad + (us - c.centre_us - c.trim_us) / (c.dir * kUsPerRad);
}

void SimServo::reset(const ServoCal& c, float rad) {
    cal_ = c;
    last_us_ = angle_to_us(c, rad);
    pos_ = target_ = us_to_angle(c, last_us_);
}

void SimServo::command_us(int us) {
    last_us_ = us;
    target_ = us_to_angle(cal_, us);
}

void SimServo::step(float dt, const SimServoParams& p) {
    const float err = target_ - pos_;
    if (std::fabs(err) * kUsPerRad <= p.deadband_us) return;
    const float max_step = p.max_rate * dt;
    pos_ += std::fmax(-max_step, std::fmin(max_step, err));
}

}  // namespace hexa
