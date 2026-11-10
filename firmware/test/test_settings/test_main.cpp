// Settings sent over Wi-Fi: a good blob gives its keys, anything that would be wrong in NVS or
// would strand a robot reached only over Wi-Fi refuses the whole blob, and why never shows a value.
#include <unity.h>

#include <string>
#include <vector>

#include "hexa_settings.h"

using namespace hexa;

void setUp() {}
void tearDown() {}

// The blob as push_settings.py sends it: header, lines, NUL.
static bool parse(const std::string& lines, std::vector<Setting>* out, std::string* why) {
    const std::string blob = std::string(kSettingsMagic) + lines + std::string(1, '\0');
    return parse_settings(blob.data(), blob.size(), out, why);
}

static bool refused(const std::string& lines) {
    std::vector<Setting> s;
    std::string why;
    return !parse(lines, &s, &why) && s.empty() && !why.empty();
}

static void test_good_blob_gives_its_keys() {
    std::vector<Setting> s;
    std::string why;
    TEST_ASSERT_TRUE(parse("# robot settings\nimu_level=3.41,-3.86\n\nidle_s = 60\r\n"
                           "trims=0,5,-10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,200\nsim=0\napp=base\nlow_bat_mv=6400\nsyslog_ip=192.168.4.1\nsyslog_port=5140\nyaw_hold=1.5\ntilt_hold=1.5\n"
                           "agent=192.168.4.1\nport=8888\nlink=wifi\nssid=net\npsk=12345678\nota_pw=x\n",
                           &s, &why));
    TEST_ASSERT_EQUAL(16, s.size());
    TEST_ASSERT_EQUAL_STRING("imu_level", s[0].key.c_str());
    TEST_ASSERT_EQUAL_STRING("3.41,-3.86", s[0].value.c_str());
    TEST_ASSERT_EQUAL_STRING("idle_s", s[1].key.c_str());
    TEST_ASSERT_EQUAL_STRING("60", s[1].value.c_str());  // spaces and CR trimmed
}

static void test_empty_value_removes_only_keys_with_a_default() {
    std::vector<Setting> s;
    std::string why;
    TEST_ASSERT_TRUE(parse("imu_level=\ntrims=\nidle_s=\nlow_bat_mv=\n", &s, &why));
    TEST_ASSERT_EQUAL(4, s.size());
    TEST_ASSERT_TRUE(s[0].value.empty());
    for (const char* k : {"ssid=\n", "psk=\n", "agent=\n", "ota_pw=\n", "link=\n"}) TEST_ASSERT_TRUE(refused(k));
}

static void test_gait_and_com() {
    std::vector<Setting> s;
    std::string why;
    TEST_ASSERT_TRUE(parse("gait=quad\ncom=-8.5,4\n", &s, &why));
    TEST_ASSERT_EQUAL(2, s.size());
    TEST_ASSERT_TRUE(parse("gait=\ncom=\n", &s, &why));  // removable: back to the defaults
    TEST_ASSERT_TRUE(refused("gait=tripod\ngait=quad\n"));  // a key given twice, as any key
}

static void test_bad_values_refuse_the_whole_blob() {
    TEST_ASSERT_TRUE(refused("idle_s=60\nimu_level=11,0\n"));  // the good line is not taken either
    TEST_ASSERT_TRUE(refused("imu_level=3.4\n"));
    TEST_ASSERT_TRUE(refused("imu_level=1e1,0\n"));
    TEST_ASSERT_TRUE(refused("idle_s=3601\n"));
    TEST_ASSERT_TRUE(refused("idle_s=-1\n"));
    TEST_ASSERT_TRUE(refused("yaw_hold=-1\n"));
    TEST_ASSERT_TRUE(refused("tilt_hold=6\n"));
    TEST_ASSERT_TRUE(refused("gait=hexa\n"));
    TEST_ASSERT_TRUE(refused("com=31,0\n"));
    TEST_ASSERT_TRUE(refused("com=5\n"));
    TEST_ASSERT_TRUE(refused("yaw_hold=5.5\n"));
    TEST_ASSERT_TRUE(refused("syslog_ip=192.168.4\n"));
    TEST_ASSERT_TRUE(refused("syslog_port=0\n"));
    TEST_ASSERT_TRUE(refused("low_bat_mv=5599\n"));
    TEST_ASSERT_TRUE(refused("low_bat_mv=8001\n"));
    TEST_ASSERT_TRUE(refused("low_bat_mv=6.4\n"));
    TEST_ASSERT_TRUE(refused("trims=0,0,0\n"));
    TEST_ASSERT_TRUE(refused("trims=0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,201\n"));
    TEST_ASSERT_TRUE(refused("port=0\n"));
    TEST_ASSERT_TRUE(refused("agent=192.168.4.256\n"));
    TEST_ASSERT_TRUE(refused("agent=192.168.4\n"));
    TEST_ASSERT_TRUE(refused("psk=short\n"));
    TEST_ASSERT_TRUE(refused("ssid=123456789012345678901234567890123\n"));
    TEST_ASSERT_TRUE(refused("app=web\n"));
    TEST_ASSERT_TRUE(refused("sim=yes\n"));
    TEST_ASSERT_TRUE(refused("colour=red\n"));
    TEST_ASSERT_TRUE(refused("just a line\n"));
    TEST_ASSERT_TRUE(refused("idle_s=1\nidle_s=2\n"));
    TEST_ASSERT_TRUE(refused(""));
}

static void test_link_serial_is_refused() {
    std::vector<Setting> s;
    std::string why;
    TEST_ASSERT_FALSE(parse("link=serial\n", &s, &why));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, why.find("USB"));
}

static void test_why_never_shows_a_value() {
    std::vector<Setting> s;
    std::string why;
    TEST_ASSERT_FALSE(parse("idle_s=5\npsk=sev7chr\n", &s, &why));
    TEST_ASSERT_EQUAL(std::string::npos, why.find("sev7chr"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, why.find("line 3"));  // the header is line 1
    TEST_ASSERT_FALSE(parse("hunter2hunter2\n", &s, &why));
    TEST_ASSERT_EQUAL(std::string::npos, why.find("hunter2"));
}

static void test_header_and_end_are_required() {
    std::vector<Setting> s;
    std::string why;
    const std::string no_header = "idle_s=5\n" + std::string(1, '\0');
    TEST_ASSERT_FALSE(settings_has_magic(no_header.data(), no_header.size()));
    TEST_ASSERT_FALSE(parse_settings(no_header.data(), no_header.size(), &s, &why));
    // Erased flash (0xff) ends it like a NUL; no end at all is refused.
    std::string erased = std::string(kSettingsMagic) + "idle_s=5\n" + std::string(16, '\xff');
    TEST_ASSERT_TRUE(parse_settings(erased.data(), erased.size(), &s, &why));
    const std::string open = std::string(kSettingsMagic) + "idle_s=5\n";
    TEST_ASSERT_FALSE(parse_settings(open.data(), open.size(), &s, &why));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_good_blob_gives_its_keys);
    RUN_TEST(test_empty_value_removes_only_keys_with_a_default);
    RUN_TEST(test_gait_and_com);
    RUN_TEST(test_bad_values_refuse_the_whole_blob);
    RUN_TEST(test_link_serial_is_refused);
    RUN_TEST(test_why_never_shows_a_value);
    RUN_TEST(test_header_and_end_are_required);
    return UNITY_END();
}
