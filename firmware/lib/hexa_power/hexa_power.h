// The final shutdown's low-battery trigger: a pack that stays under the floor while the robot
// stands or lies still. Walking sags the 2S pack by several hundred mV (the servos' current),
// so readings taken while it walks are not counted, and one reading at or over the floor
// starts the count again.
#pragma once

namespace hexa {

class LowBattery {
public:
    // floor_mv 0: never. readings: how many in a row (1 Hz: seconds).
    void init(int floor_mv, int readings = 30) {
        floor_mv_ = floor_mv;
        readings_ = readings;
        count_ = 0;
    }
    // One reading; mv <= 0 (no battery monitor) is not a reading. True once `readings`
    // readings in a row, all taken still, were under the floor.
    bool update(int mv, bool still) {
        if (floor_mv_ <= 0 || mv <= 0 || !still) return tripped();
        count_ = mv < floor_mv_ ? count_ + 1 : 0;
        return tripped();
    }
    bool tripped() const { return floor_mv_ > 0 && count_ >= readings_; }
    int floor_mv() const { return floor_mv_; }

private:
    int floor_mv_ = 0, readings_ = 30, count_ = 0;
};

}  // namespace hexa
