#include "hexa_sensors.h"

#include <cmath>
#include <limits>

namespace hexa {

uint32_t Rng::next() {
    s_ ^= s_ << 13;
    s_ ^= s_ >> 17;
    s_ ^= s_ << 5;
    return s_;
}

float Rng::uniform(float peak) {
    // 24 bits: exact in a float.
    const float u = static_cast<float>(next() >> 8) / 16777215.0f;  // 0 .. 1
    return (2.0f * u - 1.0f) * peak;
}

namespace {

float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

// A random walk step with the given drift (per sqrt(s)): unit-variance uniform noise.
float walk(Rng& r, float drift, float dt) { return drift * std::sqrt(dt) * r.uniform(1.7320508f); }

}  // namespace

void SimImu::init(const ImuParams& p, uint32_t seed) {
    p_ = p;
    rng_ = Rng(seed);
    // Sensor axes = body axes turned by roll about x, then pitch about y; a reading is the
    // body vector expressed in the sensor frame: m = (Ry(pitch) Rx(roll))^T.
    const float cr = std::cos(p.mount_roll), sr = std::sin(p.mount_roll);
    const float cp = std::cos(p.mount_pitch), sp = std::sin(p.mount_pitch);
    const float m[3][3] = {{cp, 0, -sp}, {sp * sr, cr, cp * sr}, {sp * cr, -sr, cp * cr}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m_[i][j] = m[i][j];
    vx_ = vy_ = ax_ = ay_ = 0;
    first_ = true;
    for (int i = 0; i < 3; ++i) gyro_bias_[i] = accel_bias_[i] = 0;
}

ImuSample SimImu::step(float dt, const BodyTwist& t) {
    // Body acceleration in the (rotating) body frame: dv/dt + w x v, band limited.
    float dvx = 0, dvy = 0;
    if (!first_ && dt > 0) {
        dvx = (t.vx - vx_) / dt;
        dvy = (t.vy - vy_) / dt;
    }
    first_ = false;
    vx_ = t.vx;
    vy_ = t.vy;
    const float k = dt / (p_.accel_tau_s + dt);
    ax_ += k * (dvx - t.wz * t.vy - ax_);
    ay_ += k * (dvy + t.wz * t.vx - ay_);

    for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] = clampf(gyro_bias_[i] + walk(rng_, p_.gyro_drift, dt), p_.gyro_bias_limit);
        accel_bias_[i] = clampf(accel_bias_[i] + walk(rng_, p_.accel_drift, dt), p_.accel_bias_limit);
    }
    const float f[3] = {ax_, ay_, p_.gravity};  // specific force: a - g, g = (0, 0, -9.81)
    const float w[3] = {0, 0, t.wz};
    float fs[3], ws[3];
    for (int i = 0; i < 3; ++i) {
        fs[i] = m_[i][0] * f[0] + m_[i][1] * f[1] + m_[i][2] * f[2];
        ws[i] = m_[i][0] * w[0] + m_[i][1] * w[1] + m_[i][2] * w[2];
        fs[i] = fs[i] * (1 + p_.scale_error) + accel_bias_[i] + rng_.uniform(p_.accel_noise);
        ws[i] = ws[i] * (1 + p_.scale_error) + gyro_bias_[i] + rng_.uniform(p_.gyro_noise);
    }
    return {fs[0], fs[1], fs[2], ws[0], ws[1], ws[2]};
}

void default_room(Wall out[kRoomWalls], float h) {
    out[0] = {h, -h, h, h};
    out[1] = {h, h, -h, h};
    out[2] = {-h, h, -h, -h};
    out[3] = {-h, -h, h, -h};
}

void SimRange::init(const RangeParams& p, uint32_t seed) {
    p_ = p;
    rng_ = Rng(seed);
}

float SimRange::measure(const Pose2& body, const Wall* walls, int n) {
    const float inf = std::numeric_limits<float>::infinity();
    const double c = std::cos(body.yaw), s = std::sin(body.yaw);
    const double ox = body.x + c * p_.x - s * p_.y;
    const double oy = body.y + s * p_.x + c * p_.y;
    double best = inf;
    const int rays = p_.rays < 1 ? 1 : p_.rays;
    for (int r = 0; r < rays; ++r) {
        const double off = rays == 1 ? 0 : p_.fov * (static_cast<double>(r) / (rays - 1) - 0.5);
        const double a = body.yaw + p_.yaw + off;
        const double dx = std::cos(a), dy = std::sin(a);
        for (int i = 0; i < n; ++i) {
            const Wall& w = walls[i];
            const double ex = w.x2 - w.x1, ey = w.y2 - w.y1;
            const double den = dx * ey - dy * ex;
            if (std::fabs(den) < 1e-12) continue;  // parallel
            const double qx = w.x1 - ox, qy = w.y1 - oy;
            const double t = (qx * ey - qy * ex) / den;  // along the ray
            const double u = (qx * dy - qy * dx) / den;  // along the wall
            if (t <= 0 || u < 0 || u > 1 || t >= best) continue;
            // Incidence: angle between the ray and the wall's normal.
            const double len = std::sqrt(ex * ex + ey * ey);
            const double cos_inc = std::fabs(dx * ey - dy * ex) / len;
            if (cos_inc < std::cos(p_.max_incidence)) continue;
            best = t;
        }
    }
    if (best > p_.max_range) return inf;
    const float m = static_cast<float>(best) + rng_.uniform(p_.noise + p_.noise_frac * static_cast<float>(best));
    if (m < p_.min_range) return -inf;
    return m > p_.max_range ? inf : m;
}

}  // namespace hexa
