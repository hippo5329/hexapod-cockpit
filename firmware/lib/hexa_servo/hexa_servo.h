// Joint angle <-> servo pulse, per-joint calibration, the simulated servo, and the frame
// the miniHexa's servo controller takes. Plain C++: the UART that carries the frames
// lives in the firmware's src/ (board_io).
#pragma once

#include <stddef.h>
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

// Pulse width per radian: the miniHexa's servos turn 280 degrees over 500-2500 us
// (the vendor's 7.143 us per degree).
constexpr float kUsPerRad = 2000.0f / (280.0f * 3.14159265f / 180.0f);

// Calibration of joint k (leg * 3 + joint, kin's leg order), from the vendor's servo
// convention: every joint at 1500 us sits at its zero, coxa and femur at 0, tibia at -90
// degrees from the femur. A longer pulse turns a coxa counter-clockwise seen from above
// on every leg; the femur and tibia servos of the left legs are mounted mirrored, so a
// longer pulse lowers a right femur and raises a left one (and the opposite for the
// tibia). The femur is held to 857-2143 us (+-90 degrees), as the vendor does. No trim.
ServoCal default_cal(int k);

// The servo controller's id of joint k: the vendor numbers legs 1-3 right rear, middle,
// front and 4-6 left front, middle, rear; id = (vendor leg - 1) * 3 + joint + 1.
uint8_t servo_id(int k);

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

// The miniHexa's servo controller (a chip on the base board that makes the 18 pulses) takes
// frames on a UART: 0x55 0x55, function 0x01, length, then command 0x01 (set servos), count,
// move time in ms (little-endian), and per servo its id and pulse in us (little-endian),
// closed by the low byte of the sum of every byte before it. The length byte counts the data
// plus 3. Servo ids 1-18 are leg*3 + joint + 1 in the vendor's leg numbering.
struct BusServo {
    uint8_t id;
    uint16_t us;
};
constexpr int kBusMinUs = 500;
constexpr int kBusMaxUs = 2500;
// No "servos off": a pulse of 0 is not "no pulse" to this controller but its minimum, 500 us (sent to
// one tibia on the robot, 2026-11-08: it swung to its end stop and held stiff). The servos hold
// whatever the board last sent for as long as the board is on; only the power switch lets them go.

// Writes the frame for n servos into out; returns its length, or 0 if it does not fit in
// cap, n is 0 or above 21, or a pulse is outside [kBusMinUs, kBusMaxUs].
size_t servo_bus_frame(const BusServo* servos, int n, uint16_t time_ms, uint8_t* out, size_t cap);

}  // namespace hexa
