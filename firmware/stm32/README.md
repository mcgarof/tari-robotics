# TARI — STM32 Firmware

Low-level embedded control firmware for the TARI (Teleoperated Autonomous Robotic Intelligence) robotic platform.

The firmware is developed in C++ using PlatformIO and the Arduino STM32 framework, with an emphasis on modularity, safety, observability, and automated testing.

## Target Hardware

- **Microcontroller:** STM32 NUCLEO-G474RE
- **IMU:** LSM6DSV16X
- **Motor controller:** ODESC 3.6 Dual
- **Motors:** 2 × 36 V / 350 W BLDC hub motors
- **Motor communication:** CAN bus

## Software Architecture

The firmware is organized into several functional modules:

| Module | Responsibilities |
|---|---|
| `can/` | STM32 FDCAN communication |
| `motor/` | ODESC protocol, motor commands, status monitoring, torque limiting |
| `imu/` | IMU hardware interface |
| `control/` | Attitude estimation, balance control, motion commands, parameter validation |
| `safety/` | Safety state management, watchdogs, tilt monitoring |
| `observability/` | Event recording, diagnostics, flight recorder and telemetry serialization |

## Verification

The current software baseline has been verified with:

- **242 passing unit tests** across 31 host-based test suites
- Successful firmware build for STM32 NUCLEO-G474RE

Hardware bring-up and physical balancing validation are ongoing.

## Build

From this directory:

```bash
pio run -e nucleo_g474re
```

## Unit Tests

Run the host-based unit tests:

```bash
pio test -e native
```

The native environment uses the Unity testing framework and allows core software components to be tested without connecting the STM32 hardware.

## Hardware Bring-up

The next development stage includes:

- ODESC configuration and motor calibration
- STM32 firmware verification on the target board
- IMU communication and sensor validation
- STM32-to-ODESC CAN communication testing
- Controlled motor actuation tests
- Electrical integration and safety verification

## System Integration

The STM32 controller is intended to manage low-level control and safety independently of the high-level Linux computer.

Higher-level functions such as teleoperation, audio/video streaming, ROS 2, and AI assistance are planned as separate software components.

The high-level system must not be required to maintain low-level safety.

## Development Status

**Experimental firmware — active development.**

Software build and host-based unit tests have passed. Full hardware integration, real-time performance, and physical balancing behavior have not yet been validated.
