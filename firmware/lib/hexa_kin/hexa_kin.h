// Hexapod body and leg kinematics. Plain C++ (no Arduino): the firmware, the native
// unit tests and the host simulator all build this same file.
//
// Frames: body frame x forward, y left, z up, origin at the body centre, metres and
// radians. Each leg has three joints: coxa (yaw about z at the hip), femur (pitch,
// 0 = horizontal, positive lifts the knee), tibia (pitch relative to the femur,
// 0 = in line with it, negative folds the foot down).
#pragma once

namespace hexa {

constexpr int kLegs = 6;
constexpr int kJointsPerLeg = 3;
constexpr int kJoints = kLegs * kJointsPerLeg;

// Leg order: right front, right middle, right rear, left rear, left middle, left
// front (clockwise from above, starting front right). Joint index = leg * 3 + j,
// j = 0 coxa, 1 femur, 2 tibia.
extern const char* const kLegNames[kLegs];
extern const char* const kJointNames[kJoints];

struct Vec3 {
    float x, y, z;
};

struct LegAngles {
    float coxa, femur, tibia;
};

struct Hip {
    float x, y;       // hip (coxa axis) position in the body frame
    float mount_yaw;  // direction the leg points at coxa = 0
};

struct Geometry {
    float coxa, femur, tibia;  // link lengths
    Hip hip[kLegs];
    float stand_z;             // foot height below the body origin when standing (negative)
    float neutral_reach;       // horizontal hip-to-foot distance of the neutral stance
};

// The miniHexa's geometry. PLACEHOLDERS until measured on the robot (open question 6
// in docs/hardware-minihexa.md): only the 240 x 234 mm outline is from the listing.
const Geometry& minihexa_geometry();

// Neutral (standing) foot position of a leg, body frame.
Vec3 neutral_foot(const Geometry& g, int leg);

// Joint angles that put the foot of `leg` at `foot` (body frame). Returns false, and
// leaves *out untouched, when the point is out of reach.
bool leg_ik(const Geometry& g, int leg, const Vec3& foot, LegAngles* out);

// Foot position (body frame) for the given joint angles.
Vec3 leg_fk(const Geometry& g, int leg, const LegAngles& a);

}  // namespace hexa
