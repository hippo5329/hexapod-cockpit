#include "hexa_kin.h"

#include <cmath>

namespace hexa {

const char* const kLegNames[kLegs] = {"rf", "rm", "rr", "lr", "lm", "lf"};

const char* const kJointNames[kJoints] = {
    "rf_coxa", "rf_femur", "rf_tibia", "rm_coxa", "rm_femur", "rm_tibia",
    "rr_coxa", "rr_femur", "rr_tibia", "lr_coxa", "lr_femur", "lr_tibia",
    "lm_coxa", "lm_femur", "lm_tibia", "lf_coxa", "lf_femur", "lf_tibia",
};

namespace {

Geometry make_minihexa() {
    Geometry g{};
    // The vendor's design figures (Hiwonder miniHexa kinematics course, D-H table; leg
    // origins and frame rotations from its firmware's interface): coxa 2.85 cm, femur
    // 5.2 cm, tibia 7.2 cm (the foot plate counted in the tibia). Body frame here: x
    // forward, y left; the vendor's is x right, y forward.
    g.coxa = 0.0285f;
    g.femur = 0.052f;
    g.tibia = 0.072f;
    constexpr float kDeg = 3.14159265f / 180.0f;
    // Hip (coxa axis) positions, and the direction each leg points at coxa = 0 (the
    // corner legs at 45 degrees, the middle legs straight out).
    const float hip[kLegs][3] = {
        {0.0565f, -0.0315f, -45.0f}, {0.0f, -0.0434f, -90.0f}, {-0.0565f, -0.0315f, -135.0f},
        {-0.0565f, 0.0315f, 135.0f}, {0.0f, 0.0434f, 90.0f},   {0.0565f, 0.0315f, 45.0f},
    };
    for (int i = 0; i < kLegs; ++i) {
        g.hip[i].x = hip[i][0];
        g.hip[i].y = hip[i][1];
        g.hip[i].mount_yaw = hip[i][2] * kDeg;
    }
    // The vendor's standing pose (femur 1409 us, tibia 1357 us on legs 1-3: femur +12.7
    // degrees, knee -110.0) puts the foot 70.1 mm out from the coxa axis and 60.0 mm
    // below it. The neutral stance is that pose.
    g.stand_z = -0.0600f;
    g.neutral_reach = 0.0701f;
    return g;
}

}  // namespace

const Geometry& minihexa_geometry() {
    static const Geometry g = make_minihexa();
    return g;
}

Vec3 neutral_foot(const Geometry& g, int leg) {
    const Hip& h = g.hip[leg];
    return {h.x + g.neutral_reach * std::cos(h.mount_yaw),
            h.y + g.neutral_reach * std::sin(h.mount_yaw), g.stand_z};
}

bool leg_ik(const Geometry& g, int leg, const Vec3& foot, LegAngles* out) {
    const Hip& h = g.hip[leg];
    const float dx = foot.x - h.x, dy = foot.y - h.y;
    const float c = std::cos(h.mount_yaw), s = std::sin(h.mount_yaw);
    const float lx = c * dx + s * dy;   // leg frame: x along the mount direction
    const float ly = -s * dx + c * dy;
    const float coxa = std::atan2(ly, lx);
    const float r = std::sqrt(lx * lx + ly * ly) - g.coxa;  // femur pivot to foot, horizontal
    const float z = foot.z;
    const float L2 = r * r + z * z;
    const float L = std::sqrt(L2);
    const float F = g.femur, T = g.tibia;
    if (L > F + T || L < std::fabs(F - T) || L < 1e-6f) return false;
    const float cos_a = (F * F + L2 - T * T) / (2.0f * F * L);
    const float cos_k = (F * F + T * T - L2) / (2.0f * F * T);
    const float alpha = std::acos(std::fmax(-1.0f, std::fmin(1.0f, cos_a)));
    const float knee = std::acos(std::fmax(-1.0f, std::fmin(1.0f, cos_k)));
    out->coxa = coxa;
    out->femur = std::atan2(z, r) + alpha;  // knee above the hip-foot line
    out->tibia = knee - static_cast<float>(M_PI);
    return true;
}

Vec3 leg_fk(const Geometry& g, int leg, const LegAngles& a) {
    const Hip& h = g.hip[leg];
    const float r = g.coxa + g.femur * std::cos(a.femur) + g.tibia * std::cos(a.femur + a.tibia);
    const float z = g.femur * std::sin(a.femur) + g.tibia * std::sin(a.femur + a.tibia);
    const float yaw = h.mount_yaw + a.coxa;
    return {h.x + r * std::cos(yaw), h.y + r * std::sin(yaw), z};
}

}  // namespace hexa
