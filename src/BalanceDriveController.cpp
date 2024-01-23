#include "BalanceDriveController.h"

#define DEBUG

#ifdef MOTOR_DRIVER

// Pins for all motor input and outputs
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 26
#define BIN2 25
#define PWMB 33
#define STBY 27

#if 0 // alternate pins that should work if issues with above
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA T0
#define PWMB T2
#define STBY 9
#endif
#endif // MOTOR_DRIVER

BalanceDriveController::BalanceDriveController() : motorLeft(STBY, AIN1, AIN2, PWMA), motorRight(STBY, BIN1, BIN2, PWMB)
{
}

BalanceDriveController::~BalanceDriveController()
{
}

void BalanceDriveController::Setup()
{
#ifdef MPU6050
    Serial.println("MPU6050 test!");

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }

#ifdef DEBUG
    Serial.println("MPU6050 Found!");
#endif

#if 0
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
#endif

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();

    mpu_gyro = mpu.getGyroSensor();
    mpu_gyro->printSensorDetails();

    delay(100);
#endif
}

void BalanceDriveController::Loop()
{
#ifdef MPU6050
    /* Get a new normalized sensor event */
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    kalmanfilter.Angle(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    kalmanfilter_angle = kalmanfilter.angle;

#ifdef DEBUG
    // serial plotter friendly format
    Serial.print("angle:");
    Serial.print(kalmanfilter.angle);
    Serial.print(",");

    Serial.print("accel.x:");
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print("accel.y:");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print("accel.z:");
    Serial.print(accel.acceleration.z);
    Serial.print(",");

    Serial.print("gyro.x:");
    Serial.print(gyro.gyro.x);
    Serial.print(",");
    Serial.print("gyro.x:");
    Serial.print(gyro.gyro.y);
    Serial.print(",");
    Serial.print("gyro.x:");
    Serial.print(gyro.gyro.z);
    Serial.println();
#endif
    delay(10);
#endif
}
