// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// QMI8658 / QMI8658A / QMI8658C (QST 6-axis, I2C): the linorobot2-cockpit driver
// (firmware/common/lib/imu/default_imu.h, QMI8658IMU, Apache-2.0), ported without its
// IMUInterface base: the same registers, settings, burst and checks.
//
//   * WHO_AM_I (0x00) reads 0x05 at 0x6B (SA0 high) or 0x6A (SA0 low, the miniHexa); both
//     are tried.
//   * +/-8 g at 4096 LSB/g, +/-1024 dps at 32 LSB/dps, both at 224.2 Hz with the on-chip
//     low-pass at 13.37 % of ODR (~30 Hz): read at 50 Hz, each read sees a settled,
//     oversampled value rather than one raw 224 Hz sample.
//   * ONE burst per sample, 0x2D..0x40: STATUSINT, STATUS0, STATUS1, the 24-bit sample
//     counter, temperature, AX..GZ. A failed burst keeps the previous sample.
#pragma once

#include <stdint.h>

class Qmi8658 {
public:
    // Finds the chip, resets and configures it, proves the configuration took and a
    // sample arrives. The bus (Wire) is the caller's, already begun.
    bool begin();
    // One burst. accel in m/s^2 (specific force), gyro in rad/s, the chip's axes.
    bool read(float accel[3], float gyro[3]);

    uint8_t address() const { return addr_; }
    uint8_t revision() const { return revision_; }
    uint32_t sample_count() const { return sample_count_; }
    float temperature_c() const { return temperature_c_; }
    uint32_t bursts_ok() const { return bursts_ok_; }
    uint32_t bursts_failed() const { return bursts_failed_; }

private:
    void write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);  // 0xFF on a NACK: a missing chip never reads as 0x05
    bool read_block(uint8_t reg, uint8_t* buf, uint8_t len);
    bool sample();

    uint8_t addr_ = 0x6B;
    uint8_t revision_ = 0;
    float accel_[3] = {};
    float gyro_[3] = {};
    uint32_t sample_count_ = 0;
    float temperature_c_ = 0.0f;
    uint32_t bursts_ok_ = 0;
    uint32_t bursts_failed_ = 0;
};
