# Active Roll Stabilization Prototype

Embedded flight software prototype for reducing unwanted roll in a student rocket using IMU feedback and four servo-actuated aerodynamic canards.

> **Project status:** Early ground-tested prototype. The system demonstrated end-to-end sensing, control-command generation, and servo actuation, but it did not reach flight validation. A subsequent technical review identified several estimator and controller issues that would need to be corrected in a future revision.

## Overview

This project was developed by the Rice Eclipse Rocketry flight software team in collaboration with the electrical and mechanical subteams.

The system used:

- A Raspberry Pi RP2350 microcontroller on a custom flight-computer PCB
- An SPI-connected inertial measurement unit
- Four PWM-controlled servos
- Four aerodynamic canards mounted around the rocket body

The initial objective was limited to stabilization around the rocket's longitudinal axis. The system was not intended to steer the rocket toward a geographic target or control its complete trajectory.

## Control Objective

The physical goal was to reduce unwanted roll rate during flight.

The IMU's Z-axis was aligned approximately with the rocket's longitudinal axis. The gyroscope therefore measured angular velocity around the controlled axis.

For a roll-rate damping controller, the desired angular velocity is:

```text
desired roll rate = 0 degrees per second
```

The corresponding control error is:

```text
roll-rate error = desired roll rate - measured roll rate
```

A controller can convert this error into a bounded canard command that produces aerodynamic torque opposing the measured rotation.

## Intended System Architecture

```text
        IMU gyroscope
              |
              v
       Roll-rate measurement
              |
              v
       Bias correction and
       signal validation
              |
              v
      Roll-rate controller
              |
              v
    Bounded servo PWM command
              |
              v
       Canard deflection
              |
              v
   Opposing aerodynamic torque
              |
              v
       Rocket roll dynamics
              |
              +---------------- feedback to IMU
```

## Hardware

The prototype system included:

- Raspberry Pi RP2350 microcontroller
- Custom flight-computer PCB
- SPI-connected IMU
- Four PWM-controlled servo motors
- Four aerodynamic canards
- Barometric pressure sensor
- High-G accelerometer
- Temperature sensor

The initial stabilization prototype primarily used the IMU and servo outputs. The additional sensors were intended for later flight-state estimation, data collection, and environmental monitoring.

## Software

The project used:

- C and C++
- Arduino-compatible RP2350 development environment
- SPI sensor communication
- PWM servo control
- Feedback control
- RocketPy simulation
- Git and GitHub

## Repository Structure

```text
IMU.h
    IMU driver interface

IMU.cpp
    SPI register configuration and sensor-reading implementation

PID_v1.h
    Modified controller interface

PID_v1.cpp
    Modified controller implementation based on the Arduino PID Library

PID_test.ino
    Main integration sketch connecting IMU measurements, control logic,
    and four servo outputs
```

## Prototype Implementation

The repository contains the first integrated control prototype.

The main application:

1. Initializes the IMU over SPI
2. Reads accelerometer and gyroscope measurements
3. Computes an estimated roll state
4. Passes the estimated state and gyroscope rate to the controller
5. Converts the controller output into an offset from the servo-neutral pulse
6. Applies basic servo bounds
7. Sends the command to four canard servos

The servo-neutral command was approximately 1,500 microseconds. Positive and negative controller outputs produced pulse-width offsets around this neutral position.

During bench testing, rotating the PCB in one direction caused the servos to produce a corrective command in the opposite direction. This verified the basic feedback polarity and end-to-end control path.

## Testing

Testing performed during the project included:

- SPI communication verification
- Manual inspection of IMU measurements
- Servo-neutral calibration
- Servo actuation testing
- Directional feedback testing
- Initial RocketPy control experiments
- Integrated PCB, servo, and canard testing
- Vehicle-mounted airflow testing

The vehicle-mounted airflow test ended when the canards failed mechanically under aerodynamic loading. The system did not proceed to flight validation.

## Results

The prototype demonstrated:

- Communication between the RP2350 and IMU over SPI
- Measurement of angular velocity around the longitudinal axis
- End-to-end command generation from IMU input to servo output
- Correct corrective-command polarity during manual rotation
- PWM control of four servo-actuated canards
- Integration across flight software, electrical, and mechanical systems
- Early simulation and ground-testing workflows

These results established the basic sensing and actuation pipeline, but they did not establish stable or flight-ready closed-loop performance.

## Engineering Review and Known Limitations

A later review identified several important limitations in the first implementation.

### Longitudinal Roll Observability

The prototype attempted to combine integrated Z-axis gyroscope measurements with an accelerometer-derived tilt angle.

An accelerometer can estimate pitch or roll when gravity changes its projection across the relevant sensor axes. However, rotation around a vertical longitudinal axis is not directly observable from gravity alone.

During powered flight, the accelerometer also measures thrust, vibration, and aerodynamic acceleration in addition to gravity. The accelerometer-derived angle used by the prototype was therefore not a valid absolute reference for longitudinal roll.

For the objective of suppressing spin, direct gyroscope roll-rate feedback is a more appropriate starting point.

### Time-Step Conversion

The integration sketch calculates its estimator time step using:

```cpp
float dt = (now - last_pid_time) / 100.0f;
```

Because `millis()` returns milliseconds, conversion to seconds should use `1000.0f`.

The prototype therefore integrated the gyroscope measurement with a time step approximately ten times larger than intended.

### Controller Update Rate

The outer application attempts to execute every 10 milliseconds, corresponding to a target rate of approximately 100 Hz.

The modified PID library retains an internal default sample period of 100 milliseconds. Since the prototype does not explicitly change that internal sample period, new controller outputs are generated at approximately 10 Hz rather than the intended 100 Hz.

The control-loop frequency was not rigorously measured on hardware.

### Gain and Unit Consistency

The inherited PID library scales its integral and derivative gains according to its configured sample period.

The modified controller then applies additional time scaling to the integral term while using a gyroscope measurement that is already expressed as angular velocity. This produces inconsistent gain units and requires refactoring before meaningful tuning can be performed.

### Saturation and Anti-Windup

The prototype applies basic limits to the final servo pulse width. However, controller output limits and integral anti-windup behavior were not fully initialized or validated.

Additional limitations include:

- No startup gyroscope-bias calibration
- No sensor stale-data detection
- No watchdog or fault-state behavior
- No neutral-command fallback after sensor failure
- No measured execution-time or jitter analysis
- No validated aerodynamic control-authority model
- No hardware-in-the-loop validation
- No flight testing

## Recommended Next Revision

A second revision would begin with a simpler and more physically appropriate roll-rate damping controller.

```cpp
corrected_rate = measured_gyro_z - estimated_gyro_bias;
rate_error = desired_rate - corrected_rate;
control_output = rate_gain * rate_error;
```

The control output would then be converted into a bounded servo command:

```cpp
servo_command = servo_neutral + control_output;
servo_command = constrain(
    servo_command,
    minimum_servo_command,
    maximum_servo_command
);
```

The revised system should include:

- Direct gyroscope roll-rate feedback
- Stationary prelaunch gyroscope-bias calibration
- A measured fixed-period control task
- Consistent physical units throughout the controller
- Explicit actuator saturation
- Integral anti-windup if integral control is added
- Sensor range and validity checking
- Stale-data detection
- Watchdog and neutral-command fallback behavior
- Timestamped telemetry and controller logging
- Higher-fidelity aerodynamic simulation
- Hardware-in-the-loop testing
- Measured execution time and scheduling jitter
- Structurally validated canards and mounting hardware
- Controlled ground testing before flight integration

## Contributions and Attribution

This was a collaborative Rice Eclipse Rocketry project and should not be interpreted as the work of a single developer.

### Repository Maintainer Contributions

The repository maintainer served as Flight Software Team Lead. Primary contributions included:

- Contributing to the feedback-controller design and adaptation
- Integrating IMU measurements with the control application
- Connecting controller output to PWM servo commands
- Implementing and testing the end-to-end sensing and actuation pipeline
- Performing bench and integrated system testing
- Coordinating software integration with the electrical and mechanical subteams
- Reviewing the original implementation and identifying estimator, timing, and controller limitations

### IMU Driver

The low-level IMU register initialization and portions of the SPI sensor driver were developed by another team member. They are included in this repository as part of the integrated team prototype.

Repository ownership and commit history should not be interpreted as sole authorship of those components.

### PID Library

`PID_v1.cpp` and `PID_v1.h` were adapted from Brett Beauregard's Arduino PID Library, Version 1.2.1:

https://github.com/br3ttb/Arduino-PID-Library

The original library is distributed under the MIT License. The version in this repository was modified to accept roll-angle and gyroscope-rate measurements directly.

## Project Status

Development paused after the initial prototype and ground-testing phase.

This repository is retained as:

- A record of the first integrated sensing, control, and actuation prototype
- An engineering retrospective on the prototype's limitations
- A reference architecture for a future roll-rate stabilization system
- An example of the transition from an initial hardware demonstration to a more rigorous controls design

The repository should not be used as flight-ready control software without substantial redesign, verification, and testing.
