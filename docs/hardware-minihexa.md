# Hiwonder miniHexa (Standard Kit)

The first robot of this project. Figures below come from the vendor's and resellers' listings (2026-11) and are
**not yet checked on a robot**; each line that a measurement confirms gets marked `(measured)`.

| item | listed | notes |
|---|---|---|
| frame | 18 DOF, 6 legs × 3 joints (coxa, femur, tibia), aluminium alloy | 240 × 234 × 84 mm, 0.58 kg |
| servos | 18 × HPD-0127 mini "anti-stall" servo, PWM | listed 1.8 kg·cm stall, 0.05 s/60° at 7.4 V, clutch protected; no position feedback expected |
| controller | Hiwonder ESP32 board: 21 PWM servo ports, 3 × I2C, 2 × GPIO, serial, buzzer, sound sensor, user button | ESP32 variant to identify (chip id on arrival) |
| IMU | QMI8658A 6-axis on the board | no magnetometer: yaw drifts without an anchor |
| range | ultrasonic module (I2C) | only exteroceptive sensor in the Standard Kit |
| power | 2 × 18650 (2S, 7.4 V nominal, 8.4 V full), 2200 mAh | listed runtime 60 min |
| expansion | AI vision (ESP32-S3 camera module), WonderEcho voice, touch, dot matrix | vision and voice are in the Advanced Kit, not the Standard |
| vendor software | Arduino sketches, PC posture/action editor, phone app | open source code and schematics promised |

Not to be confused with the **miniHexa Pro**, which has a different controller module (ESP32-P4 "WonderLLM");
its firmware is not for this board.

## Open questions for the first bench session

1. Exact ESP32 variant and flash size; USB-serial chip (CP210x/CH340) and its USB id.
2. Servo PWM: frequency, pulse range, centre and direction per joint; driven straight from GPIO (LEDC) or through
   a PWM expander (PCA9685) on I2C.
3. Servo bus voltage: from the 2S pack directly, or regulated.
4. Battery voltage sense: ADC pin and divider.
5. IMU and ultrasonic I2C addresses and buses.
6. Leg geometry: coxa/femur/tibia lengths and hip positions, measured, for the IK.
7. The vendor's per-servo deviation (trim) values: read and save them **before** replacing the vendor firmware.

## Sources

- Amazon listing B0FMF3Z5W1 (sold by LewanSoul, Hiwonder's store)
- Hiwonder miniHexa product page: https://www.hiwonder.com/collections/robotics/products/minihexa
- RobotShop Standard Kit listing:
  https://www.robotshop.com/products/hiwonder-hiwonder-minihexa-ai-hexapod-robot-with-ai-vision-voice-interaction-support-arduino-programming-sensor-expansion-standard-kit
- miniHexa Pro listing (for the differences): https://openelab.io/products/hiwonder-minihexa-pro
