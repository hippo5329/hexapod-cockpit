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
| standing pose | the vendor's standing pulses = femur +12.7°, knee −110°: the foot 70.1 mm out from the coxa axis, 60.0 mm below it. The IMU reads the body at about +3° roll, −4° pitch standing on a floor (measured) |
| trims | the vendor firmware keeps 18 per-servo "deviations" (µs) in its NVS; read them off before replacing it and carry them in the robot settings (`trims`) |
| walking | on glazed tile with the default gait the robot reaches about two thirds of the commanded turn and distance (measured: gyro and camera); the hard feet slip |

The pins and buses match Hiwonder's published sketch for the board, taken as interface facts (no code copied).

## Sources

- Amazon listing B0FMF3Z5W1 (sold by LewanSoul, Hiwonder's store)
- Hiwonder miniHexa product page: https://www.hiwonder.com/collections/robotics/products/minihexa
- RobotShop Standard Kit listing:
  https://www.robotshop.com/products/hiwonder-hiwonder-minihexa-ai-hexapod-robot-with-ai-vision-voice-interaction-support-arduino-programming-sensor-expansion-standard-kit
- miniHexa Pro listing (for the differences): https://openelab.io/products/hiwonder-minihexa-pro
- Hiwonder's miniHexa sketches (pins, buses, the servo controller's frame, servo conventions):
  https://github.com/Hiwonder/MiniHexa
- Hiwonder's miniHexa course (the leg's D-H table): https://github.com/Hiwonder-docs/miniHexa
