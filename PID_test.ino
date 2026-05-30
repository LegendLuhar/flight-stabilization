#include "PID_v1.h"
#include <Servo.h>
#include "IMU.h"

#define IMU_CS 13
#define SPI_SCK 10
#define SPI_SDI 8
#define SPI_SDO 11
#define PWM_SERVO1_PIN 16
#define PWM_SERVO2_PIN 26
#define PWM_SERVO3_PIN 27
#define PWM_SERVO4_PIN 28



//modify the servo limits (in microseconds)
#define LOWER_SERVO_LIMIT 600
#define SERVO_NEUTRAL 1500
#define UPPER_SERVO_LIMIT 2400
#define ROLL_GYRO_BIAS 0.98

// Create ICM42688 instance with SPI1 and CS pin 13
ICM42688 imu(SPI1, IMU_CS);
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
double Setpoint = 0.0;
//Specify the links and initial tuning parameters
//assuming Kp is microseconds/degree
double Kp=5.0, Ki=0.01, Kd=0.1;
PID myPID(&Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long last_pid_time;
double roll_from_accel, roll_angle, accel_x, accel_y, accel_z, gyro_z;

double pid_output_us = 0;



void setup()
{
last_pid_time = 0;
//PID setpoint angle: change as needed
Setpoint = 0;
roll_angle = 0;
Serial.begin(115200);
delay(1000);

Serial.println("PID test");
// Configure SPI pins
SPI1.setSCK(SPI_SCK);
SPI1.setTX(SPI_SDO);
SPI1.setRX(SPI_SDI);
SPI1.begin();

//configure servo outputs
servo_1.attach(PWM_SERVO1_PIN, LOWER_SERVO_LIMIT, UPPER_SERVO_LIMIT);
servo_2.attach(PWM_SERVO2_PIN, LOWER_SERVO_LIMIT, UPPER_SERVO_LIMIT);
servo_3.attach(PWM_SERVO3_PIN, LOWER_SERVO_LIMIT, UPPER_SERVO_LIMIT);
servo_4.attach(PWM_SERVO4_PIN, LOWER_SERVO_LIMIT, UPPER_SERVO_LIMIT);
servo_1.writeMicroseconds(SERVO_NEUTRAL);
servo_2.writeMicroseconds(SERVO_NEUTRAL);
servo_3.writeMicroseconds(SERVO_NEUTRAL);
servo_4.writeMicroseconds(SERVO_NEUTRAL);

// Initialize IMU
if (imu.begin())
{
Serial.println("IMU initialized successfully");
}
else
{
Serial.println("IMU initialization failed");
}

myPID.SetMode(AUTOMATIC);
Serial.println();
}

void loop()
{
unsigned long now = millis();
if (now - last_pid_time >= 10) { //~100Hz control loop
float dt = (now - last_pid_time) / 100.0f; // dt is exactly the time passed
//get the accel and gyro inputs
imu.readAll();
accel_x = imu.getAccelX();
accel_y = imu.getAccelY();
accel_z = imu.getAccelZ();
gyro_z = imu.getGyroZ();

//comput angle
roll_from_accel = atan2(accel_y,sqrt(accel_x*accel_x + accel_z*accel_z))*(180.0/PI);
roll_angle = (ROLL_GYRO_BIAS * (roll_angle +(gyro_z*dt))) + ((1-ROLL_GYRO_BIAS)*roll_from_accel);

//get PID output
if(myPID.Compute(roll_angle, gyro_z,&pid_output_us)){
int servo_out_us = (int)round(SERVO_NEUTRAL + pid_output_us);
//we should clamp this at the lower and upper servo limits first
if (servo_out_us > UPPER_SERVO_LIMIT){
  servo_out_us = UPPER_SERVO_LIMIT - 100;
}
if (servo_out_us < LOWER_SERVO_LIMIT){
  servo_out_us = LOWER_SERVO_LIMIT +100;
}

//Modify this to control all 3 servos
Serial.println(servo_out_us);
servo_1.writeMicroseconds(servo_out_us);
servo_2.writeMicroseconds(servo_out_us);
servo_3.writeMicroseconds(servo_out_us);
servo_4.writeMicroseconds(servo_out_us);

}
last_pid_time = now;
}
}