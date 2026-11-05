// Tripod gait: body velocity in, foot positions out. Plain C++, shared by the
// firmware, the tests and the simulator.
//
// Legs rf, rr, lm form tripod A and rm, lr, lf tripod B; each spends `duty` of the
// cycle on the ground. A stance foot moves against the commanded body twist; a swing
// foot lifts and flies to where it must land so that the next stance is centred on
// its neutral position. With a zero command the gait keeps cycling until every foot
// is back on its neutral position, then stands: it never stops mid-stride.
#pragma once

#include "hexa_kin.h"

namespace hexa {

// Defaults tuned on miniHexa #1 on glazed tile (2026-11-05, gyro + camera; variant F of
// seven): a slower cycle takes fewer, longer steps per metre, so fewer touchdowns skid, and
// a 0.7 duty keeps both tripods down for a fifth of the cycle at each change. Turns came
// from 0.38-0.41 of the command to 0.65-0.68, straight lines from 53 % to 67 % (the 1.0 s,
// 0.5, 25 mm, 40 mm defaults before). Top speed max_stride / (duty * period): 57 mm/s.
struct GaitParams {
    float period_s = 1.5f;      // one full cycle (both tripods step once)
    float duty = 0.7f;          // fraction of the cycle a foot is on the ground
    float lift = 0.030f;        // swing height
    float max_stride = 0.060f;  // longest stance travel of any foot
    float cmd_timeout_s = 0.5f; // a command older than this counts as zero
    // Adaptive cadence: period_s is the fastest cycle (top speed). Asked for less, the cycle slows,
    // up to max_period_s, so the stride stays near max_stride and the robot takes fewer steps
    // per metre (each step costs the servos the same, the distance it covers does not). A
    // command that moves no foot faster than min_foot_speed is zero: no stepping on the spot.
    float max_period_s = 3.0f;
    float min_foot_speed = 0.002f;  // m/s
};

struct BodyTwist {
    float vx, vy, wz;  // m/s, m/s, rad/s, body frame
};

// Heading hold and turn-rate feedback from the gyro: the feet slip, so a straight walk drifts (up
// to -0.24 rad in 20 s on tile) and a turn makes 60-65 % of its rate. The integral of (commanded -
// measured) yaw rate while walking is the heading lost; a trim of gain x that, plus a slowly learned
// steady part, is added to the gait's yaw rate (TripodGait::set_yaw_trim), capped. Reset while
// standing. Gain 0: off.
struct YawHoldParams {
    float gain = 1.0f;       // 1/s: trim (rad/s) per rad of heading lost
    float max_error = 0.3f;  // rad, the integral's cap (no wind-up against a wall)
    float max_trim = 0.25f;  // rad/s
    float learn = 0.5f;      // 1/s^2: the learned part of the trim (a steady slip needs a steady trim,
                             // which the heading term alone holds only by staying behind)
};

class YawHold {
public:
    explicit YawHold(const YawHoldParams& p = YawHoldParams()) : p_(p) {}
    void set_gain(float g) { p_.gain = g; }
    const YawHoldParams& params() const { return p_; }
    // One tick: the commanded and the measured (bias-corrected gyro) yaw rate, whether the gait
    // walks. Returns the trim.
    float update(float wz_cmd, float wz_meas, bool walking, float dt) {
        if (!walking || p_.gain <= 0) {
            err_ = learned_ = 0;
            return 0;
        }
        err_ += (wz_cmd - wz_meas) * dt;
        err_ = cap(err_, p_.max_error);
        learned_ = cap(learned_ + p_.learn * err_ * dt, p_.max_trim);
        return cap(p_.gain * err_ + learned_, p_.max_trim);
    }
    float error() const { return err_; }  // rad: the heading lost so far this walk

private:
    static float cap(float v, float m) { return v > m ? m : v < -m ? -m : v; }
    YawHoldParams p_;
    float err_ = 0, learned_ = 0;
};

class TripodGait {
public:
    void init(const Geometry* g, const GaitParams& p);

    // New parameters, taken only while the gait stands (a change mid-stride would jump the
    // feet); false, and nothing changed, otherwise or when a value is outside the checks.
    bool set_params(const GaitParams& p);
    static bool params_valid(const GaitParams& p);
    const GaitParams& params() const { return p_; }

    // New command at time `now_s`. Clamped to what the stride allows.
    void command(const BodyTwist& cmd, float now_s);

    // Advance by dt. `now_s` drives the command timeout.
    void step(float dt, float now_s);

    const Vec3& foot(int leg) const { return foot_[leg]; }
    bool in_stance(int leg) const;
    bool standing() const { return standing_; }
    // Stops that never settled and were put down by the backstop (step()): each one is a fault.
    int forced_stands() const { return forced_stands_; }

    // The twist the stance feet are moving the body by this tick (zero while standing).
    // This is what gait odometry integrates.
    BodyTwist applied() const;

    // The clamped command (what the gait is asked for, even while it settles).
    const BodyTwist& target() const { return cmd_; }
    // The cycle in use (adaptive cadence: period_s .. max_period_s).
    float period() const { return period_; }
    // The command as given (after the dead band; before the stride clamp and the yaw trim).
    const BodyTwist& commanded() const { return raw_; }
    // A yaw rate added to what the feet do while the gait walks (YawHold's correction for slip);
    // applied(), the gait's odometry, leaves it out: the trim makes the body turn as commanded,
    // so the commanded turn is what odometry should integrate.
    void set_yaw_trim(float wz) { trim_wz_ = wz; }

private:
    float leg_phase(int leg) const;
    BodyTwist clamp(const BodyTwist& c) const;
    float worst_foot_speed(const BodyTwist& c) const;
    bool settled() const;

    const Geometry* g_ = nullptr;
    GaitParams p_;
    BodyTwist raw_{0, 0, 0};  // as commanded (after the dead band)
    BodyTwist cmd_{0, 0, 0};  // + the yaw trim, clamped to what the stride allows at the current cycle
    BodyTwist odom_{0, 0, 0}; // what applied() reports: cmd_'s translation, the commanded yaw rate
    float trim_wz_ = 0;
    float period_ = 1.5f;
    float cmd_time_ = -1e9f;
    float phase_ = 0.0f;
    bool standing_ = true;
    float idle_s_ = 0;      // stepping with a zero command, since
    int forced_stands_ = 0;
    bool was_stance_[kLegs];
    Vec3 foot_[kLegs];
    Vec3 swing_from_[kLegs];
    Vec3 landing_[kLegs];
};

}  // namespace hexa
