# Hiwonder miniHexa (Standard Kit)

The first robot of this project. The table below is the vendor's and resellers' listings (2026-11); what the
first robot showed follows it, each measured line marked so.

| item | listed | notes |
|---|---|---|
| frame | 18 DOF, 6 legs × 3 joints (coxa, femur, tibia), aluminium alloy | 240 × 234 × 84 mm, 0.58 kg |
| servos | 18 × HPD-0127 mini "anti-stall" servo, PWM | listed 1.8 kg·cm stall, 0.05 s/60° at 7.4 V, clutch protected; no position feedback expected |
| controller | Hiwonder ESP32 board: 21 PWM servo ports, 3 × I2C, 2 × GPIO, serial, buzzer, sound sensor, user button | see below |
| IMU | QMI8658A 6-axis on the board | no magnetometer: yaw drifts without an anchor |
| range | ultrasonic module (I2C) | only exteroceptive sensor in the Standard Kit |
| power | 2 × 18650 (2S, 7.4 V nominal, 8.4 V full), 2200 mAh | listed runtime 60 min |
| expansion | AI vision (ESP32-S3 camera module), WonderEcho voice, touch, dot matrix | vision and voice are in the Advanced Kit, not the Standard |
| vendor software | Arduino sketches, PC posture/action editor, phone app | open source code and schematics promised |

Not to be confused with the **miniHexa Pro**, which has a different controller module (ESP32-P4 "WonderLLM");
its firmware is not for this board.

## What the first robot showed (2026-11)

| item | found |
|---|---|
| controller | ESP32-D0WD rev v1.1, 4 MB flash; USB serial CH340 (`1a86:7523`); opening the port resets the board (measured) |
| servos | not on ESP32 pins: a servo controller on the board takes frames on a UART (230400 baud, ESP32 RX 16 / TX 17) and makes the 18 pulses; 500-2500 µs over 280° ([firmware.md](firmware.md), Servos). Each moves alone on its id (measured, two cameras) |
| I2C | SDA 22, SCL 23, **100 kHz** (the ultrasonic does not answer at 400 kHz, measured); QMI8658 at 0x6A, ultrasonic at 0x77, battery monitor at 0x46, and an unidentified device at 0x7E |
| IMU | QMI8658, WHO_AM_I 5; at rest gravity 9.81 m/s² on +z, gyro noise ~0.001 rad/s at 50 Hz; a counter-clockwise turn reads positive on z (measured) |
| range | Hiwonder's I2C ultrasonic module: it times its own echo and returns millimetres in register 0 (no trigger or echo pin); steady to about ±15 mm (measured) |
| battery | the board's I2C monitor at 0x46 returns millivolts; GPIO33 has no divider on this board. 2S pack: 7.8 V at rest after delivery, ~0.25 V per 20 min standing on the floor (measured) |
| leg geometry | the vendor's design figures: coxa 28.5 mm, femur 52 mm, tibia 72 mm (foot plate included); hips at (±56.5, ±31.5) mm and (0, ±43.4) mm; corner legs out at 45°, middle legs straight out. Not yet measured with a ruler |
| standing pose | the vendor's standing pulses = femur +12.7°, knee −110°: the foot 70.1 mm out from the coxa axis, 60.0 mm below it. With the top plate level, the IMU reads +3.4° roll, −3.9° pitch: the IMU sits tilted (chip or board), not the body (measured with a level and a turn in place; the floor ≤ 0.3°). That is this robot's `imu_level` |
| trims | the vendor firmware keeps 18 per-servo "deviations" (µs) in its NVS; read them off before replacing it and carry them in the robot settings (`trims`) |
| walking | on glazed tile with the default gait the robot reaches about two thirds of the commanded turn and distance (measured: gyro and camera); the hard feet slip |

The pins and buses match Hiwonder's published sketch for the board, taken as interface facts (no code copied).

## The Advanced kit (from Hiwonder's published sources; not yet on a robot)

The Advanced kit adds modules to the same ESP32 controller. What the vendor's sketches show (facts read from
github.com/Hiwonder/MiniHexa, none of its code used):

| module | connects | what it gives |
|---|---|---|
| ESP32-S3 vision module | I2C 0x52, the controller's bus (100 kHz) | detection runs on the module, one task per module firmware: colour (reg 0x00-0x03: the largest red / green / blue / purple blob; 0x04: ids 1-4), face (0x01), line (0xA0-0xA7: per colour, image rows 0-39 and 40-79). A read is one register byte written, then 4 bytes: centre x, centre y, width, height (pixels of a 160×120 frame for colour and line, 240×240 for face; all 0: nothing). Polled every 20-50 ms. Video over Wi-Fi is a separate module firmware (its own access point, an HTTP MJPEG stream on port 81), not the detecting one |
| WonderEcho voice module | I2C 0x34 | offline speech recognition: reg 100 (0x64), 1 byte, the recognised command id (0: none); speech: 2 bytes `[type, id]` to reg 110 (0x6E), type 0x00 a command phrase or 0xFF an announcement. Polled every ~100 ms; the sketches speak at most every 1.5-3 s |
| touch | GPIO18, low = touched | |
| 2 × infrared obstacle | GPIO18 and GPIO32, low = obstacle (high = no reflection, used as an edge) | GPIO18 is shared with touch: one or the other per port |
| 16×8 LED dot matrix | TM1640-style two wire, GPIO14 clock, GPIO32 data | GPIO32 is shared with the second infrared sensor |

Board revisions: V1.0 reads the battery from the I2C monitor at 0x46, V1.1 through GPIO33 behind 100k/10k; the
fourth expansion pin is GPIO33 on V1.0 and GPIO14 on V1.1. The firmware names what answers in its boot line
(`modules: imu, sonar, vision 0x52, voice 0x34`). Planned: `/vision/detections` (the module's boxes),
`/voice/command` and a speech topic, `/touch` and `/ir` (std_msgs/Bool); each needs a publisher or
subscription more in `esp32.meta`.

## Sources

- Amazon listing B0FMF3Z5W1 (sold by LewanSoul, Hiwonder's store)
- Hiwonder miniHexa product page: https://www.hiwonder.com/collections/robotics/products/minihexa
- RobotShop Standard Kit listing:
  https://www.robotshop.com/products/hiwonder-hiwonder-minihexa-ai-hexapod-robot-with-ai-vision-voice-interaction-support-arduino-programming-sensor-expansion-standard-kit
- miniHexa Pro listing (for the differences): https://openelab.io/products/hiwonder-minihexa-pro
- Hiwonder's miniHexa sketches (pins, buses, the servo controller's frame, servo conventions):
  https://github.com/Hiwonder/MiniHexa
- Hiwonder's miniHexa course (the leg's D-H table): https://github.com/Hiwonder-docs/miniHexa
