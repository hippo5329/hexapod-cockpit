#include "hexa_ahrs.h"

#include <cmath>

namespace hexa {

namespace {

void normalize(Quat& q) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0) || !std::isfinite(n)) {
        q = Quat();
        return;
    }
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
}

}  // namespace

void Ahrs::init(const AhrsParams& p) {
    p_ = p;
    q_ = Quat();
    seeded_ = false;
}

bool Ahrs::seed(float ax, float ay, float az) {
    // A sensor at rest reads +g along the world's up, expressed in its own frame:
    // (-sin pitch, cos pitch sin roll, cos pitch cos roll) g.
    const float roll = std::atan2(ay, az);
    const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const float cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const float cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    q_.w = cr * cp;
    q_.x = sr * cp;
    q_.y = cr * sp;
    q_.z = -sr * sp;
    normalize(q_);
    return true;
}

void Ahrs::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    const float n = std::sqrt(ax * ax + ay * ay + az * az);
    const bool accel_ok = n > 0 && std::isfinite(n);
    if (!seeded_) {
        if (accel_ok) seeded_ = seed(ax, ay, az);
        return;
    }
    if (!(dt > 0) || dt > p_.max_dt) return;
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) return;

    const float w = q_.w, x = q_.x, y = q_.y, z = q_.z;
    // The gyro's rate of change of q: 1/2 q (x) (0, g).
    float dw = 0.5f * (-x * gx - y * gy - z * gz);
    float dx = 0.5f * (w * gx + y * gz - z * gy);
    float dy = 0.5f * (w * gy - x * gz + z * gx);
    float dz = 0.5f * (w * gz + x * gy - y * gx);

    if (accel_ok) {
        ax /= n;
        ay /= n;
        az /= n;
        // Error: world up as the estimate sees it in the sensor frame (the third row of
        // the rotation), minus the measured direction.
        const float f1 = 2 * (x * z - w * y) - ax;
        const float f2 = 2 * (y * z + w * x) - ay;
        const float f3 = 1 - 2 * (x * x + y * y) - az;
        // Its gradient over q: J^T f.
        float sw = -2 * y * f1 + 2 * x * f2;
        float sx = 2 * z * f1 + 2 * w * f2 - 4 * x * f3;
        float sy = -2 * w * f1 + 2 * z * f2 - 4 * y * f3;
        float sz = 2 * x * f1 + 2 * y * f2;
        const float sn = std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
        if (sn > 0) {
            // A step of fixed size beta: the filter's one tuning.
            dw -= p_.beta * sw / sn;
            dx -= p_.beta * sx / sn;
            dy -= p_.beta * sy / sn;
            dz -= p_.beta * sz / sn;
        }
    }

    q_.w += dw * dt;
    q_.x += dx * dt;
    q_.y += dy * dt;
    q_.z += dz * dt;
    normalize(q_);
}

float Ahrs::roll() const {
    return std::atan2(2 * (q_.w * q_.x + q_.y * q_.z), 1 - 2 * (q_.x * q_.x + q_.y * q_.y));
}

float Ahrs::pitch() const {
    const float s = 2 * (q_.w * q_.y - q_.z * q_.x);
    return std::asin(s > 1 ? 1 : (s < -1 ? -1 : s));
}

float Ahrs::yaw() const {
    return std::atan2(2 * (q_.w * q_.z + q_.x * q_.y), 1 - 2 * (q_.y * q_.y + q_.z * q_.z));
}

}  // namespace hexa
