// The final shutdown's low-battery trigger: only a pack under the floor for the whole count,
// with the robot still, shuts it down; walking sag and a missing monitor never do.
#include <unity.h>

#include "hexa_power.h"

using namespace hexa;

void setUp() {}
void tearDown() {}

static void test_trips_after_the_count_still_under_the_floor() {
    LowBattery b;
    b.init(6400, 30);
    for (int i = 0; i < 29; ++i) TEST_ASSERT_FALSE(b.update(6390, true));
    TEST_ASSERT_TRUE(b.update(6390, true));
    TEST_ASSERT_TRUE(b.update(6500, false));  // tripped stays tripped
}

static void test_one_reading_at_the_floor_starts_again() {
    LowBattery b;
    b.init(6400, 30);
    for (int i = 0; i < 29; ++i) b.update(6300, true);
    TEST_ASSERT_FALSE(b.update(6400, true));
    for (int i = 0; i < 29; ++i) TEST_ASSERT_FALSE(b.update(6300, true));
    TEST_ASSERT_TRUE(b.update(6300, true));
}

static void test_walking_sag_is_not_counted() {
    LowBattery b;
    b.init(6400, 30);
    for (int i = 0; i < 100; ++i) TEST_ASSERT_FALSE(b.update(6000, false));
    // Sag readings neither count nor reset: the still ones around them do.
    for (int i = 0; i < 15; ++i) b.update(6300, true);
    for (int i = 0; i < 10; ++i) b.update(6000, false);
    for (int i = 0; i < 14; ++i) TEST_ASSERT_FALSE(b.update(6300, true));
    TEST_ASSERT_TRUE(b.update(6300, true));
}

static void test_no_monitor_or_floor_zero_never_trips() {
    LowBattery b;
    b.init(6400, 3);
    for (int i = 0; i < 10; ++i) TEST_ASSERT_FALSE(b.update(-1, true));
    b.init(0, 3);
    for (int i = 0; i < 10; ++i) TEST_ASSERT_FALSE(b.update(5000, true));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_trips_after_the_count_still_under_the_floor);
    RUN_TEST(test_one_reading_at_the_floor_starts_again);
    RUN_TEST(test_walking_sag_is_not_counted);
    RUN_TEST(test_no_monitor_or_floor_zero_never_trips);
    return UNITY_END();
}
