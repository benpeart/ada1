#include "conditional.h"
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
#include <WebSocketsServer.h>

WebSocketsServer wsServer = WebSocketsServer(81);

// Plot settings
struct
{
    boolean enable = 0; // Enable sending data
    uint8_t prescaler = 4;
} plot;

#endif

#ifdef MPU6050
MPU6050_Base imu;
KalmanFilter kalmanfilter;

// Kalman Filter parameters
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

// PID parameters
float kp_balance = 55, kd_balance = 0.75;
float kp_speed = 10, ki_speed = 0.26;
float kp_turn = 2.5, kd_turn = 0.5;

// MPU6050 calibration parameters
float angle_zero = -0.7f;            // x axle angle calibration
float angular_velocity_zero = -4.1f; // x axle angular velocity calibration
#endif

// Rotary encoder state
volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;

// Exported so Xbox controller can change them
int setting_car_speed = 0;
int setting_turn_speed = 0;

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
    static int speed_control_period_count = 0;
    static int encoder_left_pulse_num_speed = 0;
    static int encoder_right_pulse_num_speed = 0;

    // Speed tracking state
    static float speed_control_output = 0;
    static float rotation_control_output = 0;
    static float speed_filter = 0;
    static float car_speed_integeral = 0;
    static float speed_filter_old = 0;
    static float pwm_left = 0;
    static float pwm_right = 0;
    // static const char balance_angle_min = -27;
    // static const char balance_angle_max = 27;
    // static const char balance_angle_min = -22;
    // static const char balance_angle_max = 22;

    int16_t ax, ay, az, gx, gy, gz;

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
        // use a complementary filter of the current speed using the encoder counts
        speed_control_period_count = 0;
        float car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
        encoder_left_pulse_num_speed = 0;
        encoder_right_pulse_num_speed = 0;
        speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
        speed_filter_old = speed_filter;

        // compute the input needed to hit our target speed
        car_speed_integeral += speed_filter;
        car_speed_integeral += -setting_car_speed;
        car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
        speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;

        // correct for yaw based on a kd_turn fraction of gyro_z
        rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
    }

    // read the IMU and compute use a Kalman Filter to compute a filtered angle
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    // use PID controller to compute the input we need to balance
    float balance_control_output = kp_balance * (kalmanfilter.angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

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

#ifdef WEB_SERVER
    static uint8_t k = 0;
    if (k == plot.prescaler)
    {
        k = 0;

        if (wsServer.connectedClients(0) > 0 && plot.enable)
        {
            float plotData[12];

            plotData[0] = kalmanfilter.angle - angle_zero;
            plotData[1] = kalmanfilter.Gyro_x - angular_velocity_zero;
            plotData[2] = ax;
            plotData[3] = ay;
            plotData[4] = az;
            plotData[5] = gx;
            plotData[6] = gy;
            plotData[7] = gz;
            plotData[8] = encoder_count_left_a;
            plotData[9] = encoder_count_right_a;
            plotData[10] = pwm_left / 255.0;
            plotData[11] = pwm_right / 255.0;
            wsServer.sendBIN(0, (uint8_t *)plotData, sizeof(plotData));
        }
    }
    k++;
#endif

#ifdef MOTOR_SERIAL_PLOTTER
    // serial plotter friendly format
    SerialPlotterOutput = true;
    DB_PRINT("correctedAngle:");
    DB_PRINT(kalmanfilter.angle - angle_zero);
    DB_PRINT(",");
    DB_PRINT("correctedGyro.x:");
    DB_PRINT(kalmanfilter.Gyro_x - angular_velocity_zero);
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
    DB_PRINT("gyro.x:");
    DB_PRINT(gx);
    DB_PRINT(",");
    DB_PRINT("gyro.y:");
    DB_PRINT(gy);
    DB_PRINT(",");
    DB_PRINT("gyro.z:");
    DB_PRINT(gz);
    DB_PRINT("motorLeft:");
    DB_PRINT(pwm_left / 255.0);
    DB_PRINT(",");
    DB_PRINT("motorRight:");
    DB_PRINT(pwm_right / 255.0);
    DB_PRINT(",");
#if 0
    DB_PRINT("angle:");
    DB_PRINT(kalmanfilter.angle);
    DB_PRINT(",");
    DB_PRINT("gyro.x:");
    DB_PRINT(kalmanfilter.Gyro_x);
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
#endif
#endif // MOTOR_SERIAL_PLOTTER

#endif // MPU6050
}

#ifdef WEB_SERVER
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        DB_PRINTF("[%u] Disconnected!\n", num);
        break;

    case WStype_CONNECTED:
    {
        IPAddress ip = wsServer.remoteIP(num);
        DB_PRINTF("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        //        sendConfigurationData(num);
        break;
    }

    case WStype_TEXT:
    {
        char *data = (char *)payload;
        DB_PRINTF("[%u] get Text: %s\n", num, data);

        switch (data[0])
        {
        case 's':
            switch (data[1])
            {
            case 't':
                plot.enable = true;
                break;
            case 'p':
                plot.enable = false;
                break;
            case 'r':
                plot.prescaler = atoi(data + 2);
                break;
            }
        }
        break;
    }

    case WStype_BIN:
    {
        DB_PRINTF("[%u] get binary length: %u\n", num, length);
        break;
    }
    default:
        break;
    }
}
#endif // WEB_SERVER

void BalanceDriveController_Setup(Preferences &preferences)
{
#ifdef WEB_SERVER
    wsServer.begin();
    wsServer.onEvent(webSocketEvent);
#endif // WEB_SERVER

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

    // Load our various calibration values from the 20K of EEPROM
    // Kalman Filter parameters
    dt = preferences.getFloat("dt", 0.005);
    Q_angle = preferences.getFloat("Q_angle", 0.001);
    Q_gyro = preferences.getFloat("Q_gyro", 0.005);
    R_angle = preferences.getFloat("R_angle", 0.5);
    C_0 = preferences.getFloat("C_0", 1);
    K1 = preferences.getFloat("K1", 0.05);

    // PID parameters
    kp_balance = preferences.getFloat("kp_balance", 55);
    kd_balance = preferences.getFloat("kd_balance", 0.75);
    kp_speed = preferences.getFloat("kp_speed", 10);
    ki_speed = preferences.getFloat("ki_speed", 0.26);
    kp_turn = preferences.getFloat("kp_turn", 2.5);
    kd_turn = preferences.getFloat("kd_turn", 0.5);

    // MPU6050 calibration parameters
    angle_zero = preferences.getFloat("angle_zero", -0.7);                       // x axle angle calibration
    angular_velocity_zero = preferences.getFloat("angular_velocity_zero", -4.1); // x axle angular velocity calibration
#endif                                                                           // MPU6050

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

#ifdef WEB_SERVER
    wsServer.loop();
#endif // WEB_SERVER

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
}
