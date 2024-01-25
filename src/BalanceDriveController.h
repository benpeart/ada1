#ifndef BalanceDriveController_h
#define BalanceDriveController_h

#define MPU6050
#define MOTOR_DRIVER

#ifdef MPU6050
#include <Adafruit_MPU6050.h>
#include <KalmanFilter.h>
#endif // MPU6050

#ifdef MOTOR_DRIVER
#include <TB6612FNG.h>
#endif // MOTOR_DRIVER

class BalanceDriveController
{
private:
    Adafruit_Sensor *mpu_accel, *mpu_gyro;
    KalmanFilter kalmanfilter;
    float kalmanfilter_angle;
    float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

#ifdef MOTOR_DRIVER
    Tb6612fng motorLeft;
    Tb6612fng motorRight;
//  Tb6612fng motor(STBY, AIN2, AIN1, PWMA); // Reversed forward motor direction.
#endif

public:
    Adafruit_MPU6050 mpu;

    BalanceDriveController();
    ~BalanceDriveController();
    void Setup();
    void Loop();
};

#endif // BalanceDriveController_h
