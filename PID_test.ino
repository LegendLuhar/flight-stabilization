#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "IMU.h"

// -------------------- Hardware configuration --------------------

constexpr uint8_t IMU_CS  = 13;
constexpr uint8_t SPI_SCK = 10;
constexpr uint8_t SPI_SDI = 8;
constexpr uint8_t SPI_SDO = 11;

constexpr uint8_t SERVO_1_PIN = 16;
constexpr uint8_t SERVO_2_PIN = 26;
constexpr uint8_t SERVO_3_PIN = 27;
constexpr uint8_t SERVO_4_PIN = 28;

// -------------------- Servo configuration --------------------

constexpr int SERVO_MIN_US     = 600;
constexpr int SERVO_NEUTRAL_US = 1500;
constexpr int SERVO_MAX_US     = 2400;

// Keep the initial control authority limited during bench testing.
// Increase only after verifying the mechanical system safely.
constexpr float MAX_CONTROL_US = 300.0f;

// -------------------- Control-loop configuration --------------------

constexpr uint32_t CONTROL_PERIOD_US = 10000;  // 10 ms = 100 Hz
constexpr float CONTROL_DT_S = 0.010f;

// Desired angular velocity around the rocket's longitudinal Z-axis.
constexpr float ROLL_RATE_SETPOINT_DPS = 0.0f;

// Initial placeholder gains.
// These must be tuned experimentally.
constexpr float KP = 5.0f;    // microseconds per degree/second
constexpr float KI = 0.10f;   // microseconds per degree
constexpr float KD = 0.0f;    // microseconds per degree/second^2

// Limits the integral contribution to prevent windup.
constexpr float INTEGRAL_LIMIT = 200.0f;

// Number of stationary samples used to estimate gyro bias.
constexpr int GYRO_CALIBRATION_SAMPLES = 500;
constexpr uint32_t GYRO_CALIBRATION_DELAY_MS = 2;

// -------------------- Objects --------------------

ICM42688 imu(SPI1, IMU_CS);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// -------------------- Controller state --------------------

uint32_t previousControlTimeUs = 0;

float gyroZBiasDps = 0.0f;
float integralError = 0.0f;
float previousError = 0.0f;

// -------------------- Helper functions --------------------

void commandAllServos(int pulseWidthUs)
{
    pulseWidthUs = constrain(
        pulseWidthUs,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    servo1.writeMicroseconds(pulseWidthUs);
    servo2.writeMicroseconds(pulseWidthUs);
    servo3.writeMicroseconds(pulseWidthUs);
    servo4.writeMicroseconds(pulseWidthUs);
}

void commandNeutral()
{
    commandAllServos(SERVO_NEUTRAL_US);
}

float calibrateGyroZBias()
{
    Serial.println("Keep the vehicle stationary during gyro calibration.");

    float sum = 0.0f;

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; ++i)
    {
        imu.readAll();
        sum += imu.getGyroZ();
        delay(GYRO_CALIBRATION_DELAY_MS);
    }

    return sum / static_cast<float>(GYRO_CALIBRATION_SAMPLES);
}

void resetController()
{
    integralError = 0.0f;
    previousError = 0.0f;
}

// -------------------- Setup --------------------

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Roll-rate stabilization controller");

    SPI1.setSCK(SPI_SCK);
    SPI1.setTX(SPI_SDO);
    SPI1.setRX(SPI_SDI);
    SPI1.begin();

    servo1.attach(SERVO_1_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo2.attach(SERVO_2_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo3.attach(SERVO_3_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo4.attach(SERVO_4_PIN, SERVO_MIN_US, SERVO_MAX_US);

    commandNeutral();

    if (!imu.begin())
    {
        Serial.println("ERROR: IMU initialization failed.");

        // Remain safely at neutral instead of running without valid feedback.
        while (true)
        {
            commandNeutral();
            delay(100);
        }
    }

    Serial.println("IMU initialized successfully.");

    gyroZBiasDps = calibrateGyroZBias();

    Serial.print("Measured gyro Z bias: ");
    Serial.print(gyroZBiasDps, 4);
    Serial.println(" deg/s");

    resetController();
    previousControlTimeUs = micros();
}

// -------------------- Main control loop --------------------

void loop()
{
    const uint32_t currentTimeUs = micros();
    const uint32_t elapsedUs = currentTimeUs - previousControlTimeUs;

    if (elapsedUs < CONTROL_PERIOD_US)
    {
        return;
    }

    /*
     * Advance by one scheduled period instead of assigning currentTimeUs.
     * This reduces long-term schedule drift.
     */
    previousControlTimeUs += CONTROL_PERIOD_US;

    /*
     * If execution was delayed excessively, neutralize the servos and reset
     * the controller rather than applying a command using stale timing.
     */
    if (elapsedUs > 3 * CONTROL_PERIOD_US)
    {
        commandNeutral();
        resetController();
        previousControlTimeUs = currentTimeUs;

        Serial.println("WARNING: control-loop deadline missed");
        return;
    }

    // Read the current angular velocity around the longitudinal Z-axis.
    imu.readAll();

    const float measuredGyroZDps = imu.getGyroZ();
    const float correctedRollRateDps =
        measuredGyroZDps - gyroZBiasDps;

    /*
     * Negative feedback:
     *
     * If the rocket rotates positively, correctedRollRateDps is positive.
     * The error becomes negative, producing an opposing servo command.
     */
    const float errorDps =
        ROLL_RATE_SETPOINT_DPS - correctedRollRateDps;

    // Integral term with anti-windup.
    integralError += errorDps * CONTROL_DT_S;
    integralError = constrain(
        integralError,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    );

    // Discrete derivative of roll-rate error.
    const float errorDerivative =
        (errorDps - previousError) / CONTROL_DT_S;

    previousError = errorDps;

    float controlOutputUs =
        KP * errorDps +
        KI * integralError +
        KD * errorDerivative;

    controlOutputUs = constrain(
        controlOutputUs,
        -MAX_CONTROL_US,
        MAX_CONTROL_US
    );

    const int servoCommandUs = static_cast<int>(
        roundf(SERVO_NEUTRAL_US + controlOutputUs)
    );

    commandAllServos(servoCommandUs);

    // Reduce or remove printing for accurate timing measurements.
    Serial.print("rate_dps=");
    Serial.print(correctedRollRateDps, 3);
    Serial.print(", error_dps=");
    Serial.print(errorDps, 3);
    Serial.print(", output_us=");
    Serial.print(controlOutputUs, 1);
    Serial.print(", servo_us=");
    Serial.println(servoCommandUs);
}
