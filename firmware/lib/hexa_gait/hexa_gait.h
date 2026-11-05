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
};

struct BodyTwist {
    float vx, vy, wz;  // m/s, m/s, rad/s, body frame
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

    // The twist the stance feet are moving the body by this tick (zero while standing).
    // This is what gait odometry integrates.
    BodyTwist applied() const;

    // The clamped command (what the gait is asked for, even while it settles).
    const BodyTwist& target() const { return cmd_; }

private:
    float leg_phase(int leg) const;
    BodyTwist clamp(const BodyTwist& c) const;
    bool settled() const;

    const Geometry* g_ = nullptr;
    GaitParams p_;
    BodyTwist cmd_{0, 0, 0};
    float cmd_time_ = -1e9f;
    float phase_ = 0.0f;
    bool standing_ = true;
    bool was_stance_[kLegs];
    Vec3 foot_[kLegs];
    Vec3 swing_from_[kLegs];
    Vec3 landing_[kLegs];
};

}  // namespace hexa
