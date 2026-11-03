// Joint angle <-> servo pulse, per-joint calibration, and the simulated servo.
// Plain C++: the hardware output (LEDC, MCPWM or an I2C PWM expander, still unknown
// on the miniHexa board) lives in the firmware's src/, not here.
#pragma once

#include <stdint.h>

#include "hexa_kin.h"

namespace hexa {

struct ServoCal {
    int16_t centre_us = 1500;  // pulse at which the joint sits at zero_rad
    int8_t dir = 1;            // +1 or -1: which way a longer pulse turns the joint
    int16_t trim_us = 0;       // per-servo deviation (the vendor calls it "deviation")
    int16_t min_us = 500;
    int16_t max_us = 2500;
    float zero_rad = 0.0f;     // joint angle at centre_us
};

// Pulse width per radian for a 180-degree, 500-2500 us servo. HPD-0127: UNMEASURED.
constexpr float kUsPerRad = 2000.0f / 3.14159265f;

// Default calibration of a joint: coxa and femur centred at 0, tibia centred at -90
// degrees (foot straight down with the femur horizontal), all dir +1, no trim.
ServoCal default_cal(int joint);

// Pulse for a joint angle, clamped to [min_us, max_us]. *clamped (optional) reports it.
int angle_to_us(const ServoCal& c, float rad, bool* clamped = nullptr);
float us_to_angle(const ServoCal& c, int us);

// A PWM servo without feedback, as the simulator and the firmware's sim mode see it:
// it turns toward the last pulse at a bounded rate, ignores changes inside its
// deadband, and resolves the pulse to whole microseconds. Figures are a typical micro
// servo's (docs/simulation.md), not the HPD-0127's: replace them when measured.
struct SimServoParams {
    float max_rate = 10.5f;    // rad/s: the listed 0.05 s/60 deg, halved for load
    float deadband_us = 3.0f;
};

class SimServo {
public:
    void reset(const ServoCal& c, float rad);
    void command_us(int us);
    void step(float dt, const SimServoParams& p);
    float angle() const { return pos_; }

private:
    ServoCal cal_;
    float pos_ = 0.0f;
    float target_ = 0.0f;
    int last_us_ = 1500;
};

}  // namespace hexa
