// Toggle various compile time features of the Balance Drive Controller
#define DEBUG 1
#define MPU6050
#define MOTOR_DRIVER
//#define MOTOR_SERIAL_PLOTTER
#define ELAPSED_TIME_SERIAL_PLOTTER
// #define WEB_SERVER

#include "BalanceDriveController.h"
#include "pins.h"
#include "debug.h"

#ifdef MPU6050
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include <MPU6050.h>
#include "Wire.h"
#endif // MPU6050

#ifdef MOTOR_DRIVER
#include <TB6612FNG.h>
#endif // MOTOR_DRIVER

#ifdef WEB_SERVER
#include "WebServer.h"
#endif

#ifdef MPU6050
MPU6050_Base imu;
KalmanFilter kalmanfilter;

// Kalman Filter parameters
constexpr static float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

// PID parameters
constexpr static float kp_balance = 55, kd_balance = 0.75;
constexpr static float kp_speed = 10, ki_speed = 0.26;
constexpr static float kp_turn = 2.5, kd_turn = 0.5;

// MPU6050 calibration parameters (never set)
constexpr static float angle_zero = 0.65f;            // x axle angle calibration
constexpr static float angular_velocity_zero = -4.5f; // x axle angular velocity calibration
#endif

// Rotary encoder state
volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;

// Speed tracking state
float speed_control_output = 0;
float rotation_control_output = 0;
float speed_filter = 0;
int speed_control_period_count = 0;
float car_speed_integeral = 0;
float speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
float pwm_left = 0;
float pwm_right = 0;
// constexpr static char balance_angle_min = -27;
// constexpr static char balance_angle_max = 27;
constexpr static char balance_angle_min = -22;
constexpr static char balance_angle_max = 22;

#ifdef MOTOR_DRIVER
// Reverse AIN1/AIN2 and BIN1/BIN2 to reverse the direction of the motors
// Tb6612fng motors(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);
Tb6612fng motorRight(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT);
Tb6612fng motorLeft(STBY, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);
#endif

void IRAM_ATTR encoderCountRightA()
{
    // put an upper bounds on this while testing
    if (encoder_count_right_a < 24)
        encoder_count_right_a++;
}

void IRAM_ATTR encoderCountLeftA()
{
    // put an upper bounds on this while testing
    if (encoder_count_left_a < 24)
        encoder_count_left_a++;
}

void BalanceCar()
{
#ifdef MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    float kalmanfilter_angle;

#ifdef MOTOR_SERIAL_PLOTTER
    // serial plotter friendly format
    SerialPlotterOutput = true;
    DB_PRINT("encoderCountLeft:");
    DB_PRINT(encoder_count_left_a);
    DB_PRINT(",");
    DB_PRINT("encoderCountRight:");
    DB_PRINT(encoder_count_right_a);
    DB_PRINT(",");
#endif
    // update the encoder pulse count (speed)
    encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
    encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
    encoder_count_left_a = 0;
    encoder_count_right_a = 0;

    // adjust the speed and rotation inputs every 8 calls
    speed_control_period_count++;
    if (speed_control_period_count >= 8)
    {
        // calculate a filtered value of the current speed using the encoder counts
        speed_control_period_count = 0;
        float car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
        encoder_left_pulse_num_speed = 0;
        encoder_right_pulse_num_speed = 0;
        speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
        speed_filter_old = speed_filter;

        // use PID controller to compute the input needed to hit our target speed
        car_speed_integeral += speed_filter;
        car_speed_integeral += -setting_car_speed;
        car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
        speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;

        // use PID controller to compute how much input we need to turn left/right
        rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
    }

    // read the IMU and compute use a Kalman Filter to compute a filtered angle
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    kalmanfilter_angle = kalmanfilter.angle;

    // use PID controller to compute the input we need to balance
    float balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

    // final motor output is combination of inputs for balance - speed +/- rotation
    pwm_left = balance_control_output - speed_control_output - rotation_control_output;
    pwm_right = balance_control_output - speed_control_output + rotation_control_output;
    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

#ifdef MOTOR_DRIVER
    // not transitioning modes, just balancing/moving
    // motors.drive(pwm_left / 255.0, pwm_right / 255.0, 0, false);
    motorLeft.drive(pwm_left / 255.0);
    motorRight.drive(pwm_right / 255.0);
#endif

#ifdef MOTOR_SERIAL_PLOTTER
    // serial plotter friendly format
    SerialPlotterOutput = true;
    DB_PRINT("motorLeft:");
    DB_PRINT(pwm_left / 255.0);
    DB_PRINT(",");
    DB_PRINT("motorRight:");
    DB_PRINT(pwm_right / 255.0);
    DB_PRINT(",");
#if 0
    DB_PRINT("angle:");
    DB_PRINT(kalmanfilter_angle);
    DB_PRINT(",");
    DB_PRINT("correctedAngle:");
    DB_PRINT(kalmanfilter_angle - angle_zero);
    DB_PRINT(",");
    DB_PRINT("gyro.x:");
    DB_PRINT(kalmanfilter.Gyro_x);
    DB_PRINT(",");
    DB_PRINT("correctedGyro.x:");
    DB_PRINT(kalmanfilter.Gyro_x - angular_velocity_zero);
    DB_PRINT(",");
    DB_PRINT("balance:");
    DB_PRINT(balance_control_output);
    DB_PRINT(",");
    DB_PRINT("pwm_left:");
    DB_PRINT(pwm_left);
    DB_PRINT(",");
    DB_PRINT("pwm_right:");
    DB_PRINT(pwm_right);
    DB_PRINT(",");
    DB_PRINT("accel.x:");
    DB_PRINT(ax);
    DB_PRINT(",");
    DB_PRINT("accel.y:");
    DB_PRINT(ay);
    DB_PRINT(",");
    DB_PRINT("accel.z:");
    DB_PRINT(az);
    DB_PRINT(",");
    DB_PRINT("gyro.y:");
    DB_PRINT(gy);
    DB_PRINT(",");
    DB_PRINT("gyro.z:");
    DB_PRINT(gz);
#endif
#endif // MOTOR_SERIAL_PLOTTER

#endif // MPU6050
}

void BalanceDriveController_Setup()
{
#ifdef MPU6050
    Wire.begin();
    imu.initialize();
    if (!imu.testConnection())
    {
        DB_PRINTLN("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }

    DB_PRINTLN("MPU6050 Found!");
#endif // MPU6050

#ifdef MOTOR_DRIVER
    // setup the motors and attach the rotary encoder counters
    // motors.begin();
    motorLeft.begin();
    motorRight.begin();

    // setup the motor hall effect sensors we use to detect out speed
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLDOWN);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderCountRightA, CHANGE);
#endif
}

void BalanceDriveController_Loop()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();

    // only run every 5ms
    if ((currentTime - lastTime) < 5)
        return;

#ifdef ELAPSED_TIME_SERIAL_PLOTTER
    SerialPlotterOutput = true;
    DB_PRINT("ElapsedTime:");
    DB_PRINT(currentTime - lastTime);
    DB_PRINT(",");
#endif
    lastTime = currentTime;

    BalanceCar();
#ifdef WEB_SERVER
    webserver_loop(mpu);
#endif
}
