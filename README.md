# Active Flight Stabilization

Prototype embedded flight software for active roll stabilization of a student rocket using servo-actuated canards.

The system was developed by the Rice Eclipse Rocketry flight software team in collaboration with the electrical and mechanical subteams. The prototype used an RP2350-based custom PCB, an IMU connected over SPI, and four PWM-controlled servos mounted to aerodynamic canards.

## Project Goal

The goal of the project was to reduce unwanted rotation around the rocket's longitudinal axis by sensing roll rate and commanding the canards to generate an opposing aerodynamic torque.

The first prototype focused only on roll stabilization. It was not designed to steer the rocket toward a geographic target or command its overall flight trajectory.


## System Architecture

```text
IMU gyroscope
      |
      v
Roll-rate measurement
      |
      v
Feedback controller
      |
      v
Servo PWM commands
      |
      v
Canard deflection
      |
      v
Opposing aerodynamic roll torque
      |
      v
Rocket roll dynamics
      |
      +---------------- feedback to IMU
```

The IMU's Z-axis was aligned with the rocket's longitudinal axis.

The gyroscope measured angular velocity around Z. The controller compared the measured roll rate with a desired roll rate of zero and commanded the servos to oppose the measured rotation.

## Hardware

- Raspberry Pi RP2350 microcontroller
- Custom flight-computer PCB
- SPI-connected inertial measurement unit
- Four PWM-controlled servo motors
- Four aerodynamic canards
- Barometer
- High-G accelerometer
- Temperature sensor

The first physical prototype primarily used the IMU and servos. The additional sensors were planned for later flight-state estimation and environmental monitoring.

## Software

- C and C++
- Arduino-compatible RP2350 environment
- SPI sensor communication
- PWM servo control
- Feedback control
- RocketPy simulation
- Git and GitHub

## Control Approach

The prototype used gyroscope measurements around the longitudinal Z-axis to determine roll direction and roll rate.

The desired roll rate was zero:

```text
roll-rate error = desired roll rate - measured roll rate
```

The controller converted this error into a servo pulse-width command. The servo neutral point was approximately 1,500 microseconds, with positive and negative offsets commanding canard motion in opposite directions.

During bench testing, rotating the PCB clockwise around the longitudinal axis caused the canards to command a counterclockwise correction, and rotating it in the opposite direction reversed the command. This verified the feedback polarity of the prototype.

## Testing

Testing included:

- Manual bench testing of IMU measurements
- SPI communication verification
- Servo-neutral and actuation testing
- Directional feedback testing
- Initial RocketPy simulation experiments
- Vehicle-mounted airflow testing

The vehicle-mounted test ended when the canards failed mechanically under aerodynamic loading. The project did not reach flight validation.

## Results

The prototype demonstrated:

- Successful SPI communication with the IMU
- Measurement of angular velocity around the longitudinal axis
- Closed-loop command generation from sensor input to servo actuation
- Correct directional response to clockwise and counterclockwise rotation
- Integration of the PCB, IMU, servos, and mechanical canard assembly

## Current Limitations

This repository represents an early prototype rather than flight-ready software.

Known limitations include:

- The controller was not fully tuned
- The control-loop rate was not rigorously measured on hardware
- The aerodynamic response was not fully characterized
- The system did not complete flight testing
- Mechanical canard strength was insufficient during the initial airflow test
- Sensor-failure handling and watchdog behavior were incomplete
- The original estimator and control implementation require further review

## Future Improvements

- Refactor the controller into a clearly timed 100 Hz task
- Measure execution time, jitter, and deadline misses
- Add gyroscope bias calibration
- Add sensor validity and stale-data checks
- Add neutral-command fallback behavior
- Implement actuator saturation and integral anti-windup
- Develop a higher-fidelity aerodynamic model
- Add hardware-in-the-loop testing
- Strengthen and validate the mechanical canard assembly
- Conduct controlled ground testing before flight integration

## Repository Structure

```text
IMU.h
    IMU driver interface

IMU.cpp
    SPI communication and sensor-reading implementation

PID_v1.h
    Controller interface

PID_v1.cpp
    Controller implementation

PID_test.ino
    Main integration sketch for IMU input, control, and servo output
```

## Project Status

Development paused after the initial prototype and ground-testing phase. The repository is retained as a record of the sensing, control, and actuation architecture and as a foundation for future revisions.
