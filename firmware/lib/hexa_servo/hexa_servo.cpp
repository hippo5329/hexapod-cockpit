#include "hexa_servo.h"

#include <cmath>

namespace hexa {

ServoCal default_cal(int k) {
    ServoCal c;
    const int joint = k % kJointsPerLeg;
    const bool left = k / kJointsPerLeg >= kLegs / 2;  // lr, lm, lf
    if (joint == 1) {
        c.dir = left ? 1 : -1;
        c.min_us = 857;
        c.max_us = 2143;
    } else if (joint == 2) {
        c.dir = left ? -1 : 1;
        c.zero_rad = -1.57079633f;
    }
    return c;
}

uint8_t servo_id(int k) {
    static const uint8_t kVendorLeg[kLegs] = {3, 2, 1, 6, 5, 4};  // rf rm rr lr lm lf
    return static_cast<uint8_t>((kVendorLeg[k / kJointsPerLeg] - 1) * kJointsPerLeg + k % kJointsPerLeg + 1);
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

size_t servo_bus_frame(const BusServo* servos, int n, uint16_t time_ms, uint8_t* out, size_t cap) {
    if (n < 1 || n > 21) return 0;
    const size_t data_len = 4 + 3 * static_cast<size_t>(n);
    const size_t len = 4 + data_len + 1;
    if (len > cap) return 0;
    out[0] = 0x55;
    out[1] = 0x55;
    out[2] = 0x01;  // function: PWM servo control
    out[3] = static_cast<uint8_t>(data_len + 3);
    out[4] = 0x01;  // command: set servos
    out[5] = static_cast<uint8_t>(n);
    out[6] = time_ms & 0xff;
    out[7] = time_ms >> 8;
    for (int i = 0; i < n; ++i) {
        if (servos[i].us < kBusMinUs || servos[i].us > kBusMaxUs) return 0;
        out[8 + 3 * i] = servos[i].id;
        out[9 + 3 * i] = servos[i].us & 0xff;
        out[10 + 3 * i] = servos[i].us >> 8;
    }
    unsigned sum = 0;
    for (size_t i = 0; i + 1 < len; ++i) sum += out[i];
    out[len - 1] = sum & 0xff;
    return len;
}

}  // namespace hexa
