# TARI — Development Roadmap

TARI (Teleoperated Autonomous Robotic Intelligence) is an experimental two-wheeled self-balancing robotics platform.

Development follows an incremental approach, progressing from embedded control and safety to hardware integration, teleoperation, ROS 2 experimentation, and AI-enabled capabilities.

This roadmap describes planned milestones and current development status. Dates and priorities may evolve as hardware integration progresses.

## Milestone 1 — Embedded Software Foundation

**Status: Software baseline implemented; hardware validation pending**

- [x] Modular STM32 firmware architecture
- [x] CAN communication and ODESC motor-control software
- [x] Motor status and heartbeat handling
- [x] Attitude estimation and balance-control software
- [x] Safety state management and watchdog logic
- [x] Command validation and timeout handling
- [x] Diagnostic and observability components
- [x] Host-based automated unit tests
- [x] Successful STM32 firmware build
- [ ] Full validation on target hardware

**Verification baseline:** 242 passing unit tests across 31 test suites.

## Milestone 2 — Hardware Bring-up

**Status: Planned / in preparation**

- [ ] Configure and calibrate the ODESC motor controller
- [ ] Verify motor direction and torque response
- [ ] Validate IMU communication and sensor measurements
- [ ] Verify STM32-to-ODESC CAN communication
- [ ] Validate motor-controller telemetry and fault reporting
- [ ] Perform initial controlled motor actuation tests
- [ ] Verify hardware safety behavior

## Milestone 3 — System Integration and Balancing

**Status: Planned**

- [ ] Complete electrical wiring and mechanical integration
- [ ] Develop PC-based configuration and diagnostic tools
- [ ] Implement live telemetry visualization
- [ ] Tune attitude estimation and balance-control parameters
- [ ] Perform initial supported balancing experiments
- [ ] Progress toward unsupported self-balancing
- [ ] Validate watchdogs, fault handling, and safe-state transitions on hardware

## Milestone 4 — High-Level Robotics and Telepresence

**Status: Planned**

- [ ] Integrate a Linux-based high-level computer
- [ ] Implement communication between Linux and STM32
- [ ] Develop remote teleoperation interfaces
- [ ] Integrate audio and video streaming
- [ ] Experiment with ROS 2 integration
- [ ] Develop robot telemetry and data collection capabilities
- [ ] Validate communication-loss behavior

## Milestone 5 — AI and Autonomous Capabilities

**Status: Exploratory / planned**

- [ ] Integrate an interactive AI assistant
- [ ] Experiment with visual perception
- [ ] Explore autonomous behaviors
- [ ] Evaluate robotics data collection for AI experimentation
- [ ] Investigate additional Physical AI capabilities

## Engineering Principles

Throughout development, TARI emphasizes:

- Separation of low-level safety and high-level processing
- Modular software architecture and testability
- Observable and diagnosable behavior
- Incremental hardware validation
- Reproducible builds and automated tests
- Safe behavior under communication failures

## Current Focus

The immediate priorities are ODESC configuration, motor calibration, IMU verification, and STM32-to-ODESC CAN integration.

Physical balancing performance and complete system-level safety remain to be validated.
