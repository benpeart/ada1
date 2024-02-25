#include "conditional.h"
#include "BalanceDriveController.h"
#include "pins.h"
#include "debug.h"

#ifdef MPU6050
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include <MPU6050.h>
#include "Wire.h"
#include <PID_v1.h>
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

#endif // WEB_SERVER

DriveMode drive_mode = MODE_PARKED;
float speed;
float steer;

#ifdef MPU6050
MPU6050_Base imu;
KalmanFilter kalmanfilter;

// Kalman Filter parameters
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

// The maximum angle we can recover from; anything greater than this we enter MODE_FALLEN
#define BALANCE_ANGLE_MAX 22

// We're going to use 3 PIDs to control the bot:
//      PID_Angle keeps the robot balanced at the given angle.
//      PID_Speed determines the speed and direction of the robot. We will use Proportional On Measurement
//      PID_Steer determines the direction the robot will turn/drive.
//
// The idea behind PID_Speed and PID_Steer is that they ensure the robot smoothly transitions
// between requested states so that it doesn't fall. They will also let us hold a constant speed and
// turn rate if/when I add ROS 2 SLAM/Navigation support.
//
// The PID_Angle setpoint is set by PID_Speed when balancing/driving.

// Define PID variables
#define PID_SAMPLE_TIME 5

PID pidAngle(0, 0, 0, 0, 0, 0, DIRECT);         // used to keep the robot balanced
PID pidSpeed(0, 0, 0, 0, 0, 0, DIRECT, P_ON_M); // Proportional On Measurement so we don't overshoot our speeds
PID pidSteer(0, 0, 0, 0, 0, 0, DIRECT, P_ON_M); // Proportional On Measurement so we don't overshoot our turning
#endif

// Rotary encoder state
volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;

// pwm_left/right range from +-255.0. motorRight/Left range from +-1.0
static float pwm_left = 0;
static float pwm_right = 0;

#ifdef MOTOR_DRIVER
// Reverse AIN1/AIN2 and BIN1/BIN2 to reverse the direction of the motors
// Tb6612fng motors(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);
Tb6612fng motorRight(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT);
Tb6612fng motorLeft(STBY, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);
#endif

void IRAM_ATTR encoderCountRightA()
{
    encoder_count_right_a++;
}

void IRAM_ATTR encoderCountLeftA()
{
    encoder_count_left_a++;
}

#ifdef WEB_SERVER

void SendDriveMode(uint8_t num)
{
    if (wsServer.connectedClients(0) <= 0)
        return;

    switch (drive_mode)
    {
    case MODE_PARKED:
        wsServer.sendTXT(num, "mp");
        break;

    case MODE_STANDING_UP:
        wsServer.sendTXT(num, "ms");
        break;

    case MODE_PARKING:
        wsServer.sendTXT(num, "mk");
        break;

    case MODE_DRIVE:
        wsServer.sendTXT(num, "md");
        break;

    case MODE_FALLEN:
        wsServer.sendTXT(num, "mf");
        break;

    case MODE_CALIBRATION:
        wsServer.sendTXT(num, "mc");
        break;
    }
}

void SendPID(uint8_t num)
{
    char wBuf[63];

    if (wsServer.connectedClients(0) <= 0)
        return;

    sprintf(wBuf, "kp%.4f", pidAngle.GetKp());
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "ki%.4f", pidAngle.GetKi());
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "kd%.4f", pidAngle.GetKd());
    wsServer.sendTXT(num, wBuf);
}

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

        // send config data to webSocket client
        SendDriveMode(num);
        SendPID(num);
        break;
    }

    case WStype_TEXT:
    {
        char *data = (char *)payload;
        DB_PRINTF("[%u] get Text: %s\n", num, data);

        switch (data[0])
        {
        case 'm':
            switch (data[1])
            {
            case 'p':
                BalanceDriveController_SetMode(MODE_PARKED);
                break;
            case 's':
                BalanceDriveController_SetMode(MODE_STANDING_UP);
                break;
            case 'k':
                BalanceDriveController_SetMode(MODE_PARKING);
                break;
            case 'd':
                BalanceDriveController_SetMode(MODE_DRIVE);
                break;
            case 'f':
                BalanceDriveController_SetMode(MODE_FALLEN);
                break;
            case 'c':
                BalanceDriveController_SetMode(MODE_CALIBRATION);
                break;
            }
            break;

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
            break;

        case 'k':
        {
            float val = atof(data + 2);
            switch (data[1])
            {
            case 'p':
                pidAngle.SetTunings(val, pidAngle.GetKi(), pidAngle.GetKd());
                DB_PRINTF("Set Kp to [%f]\n", val);
                break;
            case 'i':
                pidAngle.SetTunings(pidAngle.GetKp(), val, pidAngle.GetKd());
                DB_PRINTF("Set Ki to [%f]\n", val);
                break;
            case 'd':
                pidAngle.SetTunings(pidAngle.GetKp(), pidAngle.GetKi(), val);
                DB_PRINTF("Set Kd to [%f]\n", val);
                break;
            }
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

    // Load our various calibration values from the 20K of EEPROM in the ESP32

    // Kalman Filter parameters
    dt = preferences.getFloat("dt", 0.005);
    Q_angle = preferences.getFloat("Q_angle", 0.001);
    Q_gyro = preferences.getFloat("Q_gyro", 0.005);
    R_angle = preferences.getFloat("R_angle", 0.5);
    C_0 = preferences.getFloat("C_0", 1);
    K1 = preferences.getFloat("K1", 0.05);

    // Initialize PID parameters
    pidAngle.Setpoint = 0;
    pidAngle.SetTunings(preferences.getFloat("Angle_kp", 8.0),
                        preferences.getFloat("Angle_ki", 1),
                        preferences.getFloat("Angle_kd", 0.075)); // aggressive
#ifdef CONSERVATIVE
    pidAngle.SetTunings(preferences.getFloat("Angle__kp", 1.0),
                        preferences.getFloat("Angle__ki", 0.05),
                        preferences.getFloat("Angle__kd", 0.25)); // conservative
#endif
    pidAngle.SetSampleTime(PID_SAMPLE_TIME);
    pidAngle.SetOutputLimits(-255, 255);
    pidAngle.SetMode(MANUAL);

#endif // MPU6050

#ifdef MOTOR_DRIVER
    // setup the motors and attach the rotary encoder counters
    // motors.begin();
    motorLeft.begin();
    motorRight.begin();
#endif

    // setup the motor hall effect sensors we use to detect our speed
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLDOWN);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderCountRightA, CHANGE);
}

// calculate the inputs necessary for the motors to fulfull the needed speed, rotation, balance requests
void UpdateMotors()
{
#ifdef MPU6050

#ifdef SPEED
    static int encoder_left_pulse_num_speed = 0;
    static int encoder_right_pulse_num_speed = 0;

    // Speed tracking state
    static float speed_control_output = 0;
    static float rotation_control_output = 0;
    static float speed_filter = 0;
    static float car_speed_integeral = 0;
    static float speed_filter_old = 0;
    // static const char balance_angle_min = -27;
    // static const char balance_angle_max = 27;

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
#endif // SPEED

    // final motor output is combination of inputs for balance - speed +/- rotation
    pwm_left = -pidAngle.Output * 20;  // - speed_control_output - rotation_control_output;
    pwm_right = -pidAngle.Output * 20; // - speed_control_output + rotation_control_output;
    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

#ifdef MOTOR_DRIVER
    motorLeft.drive(pwm_left / 255.0);
    motorRight.drive(pwm_right / 255.0);
#endif

#endif // MPU6050
}

//
// Compute the angle of the robot and transition to any necessary new state (stood up, parked, or fallen).
// Update pidAngle and if we are driving and no state changes were detected, update the motors to keep us driving and balancing.
//
void BalanceDriveController_Loop()
{
#ifdef WEB_SERVER
    wsServer.loop();
#endif // WEB_SERVER

    // only run every PID_SAMPLE_TIME ms
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();

    if ((currentTime - lastTime) < PID_SAMPLE_TIME)
        return;
#ifdef ELAPSED_TIME_SERIAL_PLOTTER
    SerialPlotterOutput = true;
    DB_PRINT("ElapsedTime:");
    DB_PRINT(currentTime - lastTime);
    DB_PRINT(",");
#endif
    lastTime = currentTime;

#ifdef MPU6050
    // read the IMU and compute use a Kalman Filter to compute a filtered angle for the robot
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    // calculate the correction needed to balance
    pidAngle.Input = kalmanfilter.angle;
    pidAngle.Compute();

    switch (drive_mode)
    {
    case MODE_PARKED:
        break;

    case MODE_STANDING_UP:
        // Detect if robot is balanced enough that we can transition to driving.
        // Ensure we are beyond BALANCE_ANGLE_MAX enough that we don't transition to DRIVE then detect a fall.
        if (abs(kalmanfilter.angle) < BALANCE_ANGLE_MAX / 2)
        {
            BalanceDriveController_SetMode(MODE_DRIVE);
            break;
        }
        break;

    case MODE_DRIVE:
        // If we exeed BALANCE_ANGLE_MAX in the positive direction, we will land on the leg and transition to MODE_PARKED
        if (kalmanfilter.angle > BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
            break;
        }

        // If we exceed BALANCE_ANGLE_MAX in the negative direction, we have FALLEN
        if (kalmanfilter.angle < -BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_FALLEN);
            break;
        }

        UpdateMotors();
        break;

    case MODE_PARKING:
    case MODE_FALLEN:
        // If we exeed BALANCE_ANGLE_MAX in the positive direction, we will land on the leg and transition to MODE_PARKED
        if (kalmanfilter.angle > BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
            break;
        }
        break;
    }
#ifdef WEB_SERVER
    static uint8_t k = 0;
    if (k == plot.prescaler)
    {
        k = 0;

        if (wsServer.connectedClients(0) > 0 && plot.enable)
        {
            float plotData[14];

            plotData[0] = pidAngle.Input;
            plotData[1] = pidAngle.Output;
            plotData[2] = kalmanfilter.angle;
            plotData[3] = kalmanfilter.Gyro_x;
            plotData[4] = ax;
            plotData[5] = ay;
            plotData[6] = az;
            plotData[7] = gx;
            plotData[8] = gy;
            plotData[9] = gz;
            plotData[10] = encoder_count_left_a;
            plotData[11] = encoder_count_right_a;
            plotData[12] = pwm_left;
            plotData[13] = pwm_right;
            wsServer.sendBIN(0, (uint8_t *)plotData, sizeof(plotData));
        }
    }
    k++;
#endif

#ifdef MOTOR_SERIAL_PLOTTER
    // serial plotter friendly format
    SerialPlotterOutput = true;
    DB_PRINT("correctedAngle:");
    DB_PRINT(kalmanfilter.angle);
    DB_PRINT(",");
    DB_PRINT("correctedGyro.x:");
    DB_PRINT(kalmanfilter.Gyro_x);
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

#endif
}

void BalanceDriveController_SetMode(DriveMode newDriveMode)
{
    if (newDriveMode == drive_mode)
        return;

    // prevent invalid transitions
    if ((drive_mode == MODE_FALLEN) && newDriveMode != MODE_PARKED)
        return;

    switch (newDriveMode)
    {
    case MODE_PARKED: // robot is parked on its leg so it can later stand
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_PARKED");
        pidAngle.SetMode(MANUAL); // we only want the PID running when we're trying to balance
        pwm_left = pwm_right = 0;
#ifdef MOTOR_DRIVER
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
#endif
        break;

    case MODE_STANDING_UP: // robot is in the process of standing
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_STANDING_UP");
        pidAngle.SetMode(MANUAL); // we only want the PID running when we're trying to balance
        pwm_left = pwm_right = 255;
#ifdef MOTOR_DRIVER
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
#endif
        break;

    case MODE_PARKING: // robot is in the process of parking
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_PARKING");
        pidAngle.SetMode(MANUAL); // we only want the PID running when we're trying to balance
        pwm_left = pwm_right = -128;
#ifdef MOTOR_DRIVER
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
#endif
        break;

    case MODE_DRIVE: // robot is balancing and able to be driven
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_DRIVE");
        pidAngle.SetMode(AUTOMATIC); // we only want the PID running when we're trying to balance
        break;

    case MODE_FALLEN: // robot has fallen and can't get up
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_FALLEN");
        pidAngle.SetMode(MANUAL); // we only want the PID running when we're trying to balance
        pwm_left = pwm_right = 0;
#ifdef MOTOR_DRIVER
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
#endif
        break;

    case MODE_CALIBRATION: // calculate a new calibration and store it in the MPU6050
        // tell the web page we're calibrating then turn off pidAngle and motors just to be sure we aren't moving
        drive_mode = newDriveMode;
        SendDriveMode(0);
        pidAngle.SetMode(MANUAL);
        pwm_left = pwm_right = 0;
#ifdef MOTOR_DRIVER
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
#endif
        // now actually run the calibration then switch us to MODE_PARKED
        DB_PRINTLN("PID tuning - each dot = 100 readings");
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        DB_PRINTLN(" Calibration complete and stored in MPU6050");
        newDriveMode = MODE_PARKED;
        break;
    }

    drive_mode = newDriveMode;
#ifdef WEB_SERVER
    SendDriveMode(0);
#endif
}

void BalanceDriveController_SetVelocity(float newSpeed, float newSteer)
{
    speed = newSpeed;
    steer = newSteer;
}
