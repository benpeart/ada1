#include "conditional.h"
#include "BalanceDriveController.h"
#include "pins.h"
#include "debug.h"
#include "main.h"

#ifdef MPU6050
#ifdef KALMANFILTER
#include "KalmanFilter.h"
#endif // KALMANFILTER
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#endif // MPU6050

#ifdef MOTOR_DRIVER
#include <TB6612FNG.h>
#else
class Tb6612fngLedc
{
public:
    Tb6612fngLedc(int pin, int chan, int freq, int range){};
    void begin(){};
    void write(float value){};
};

class Tb6612fngMotor
{
public:
    Tb6612fngMotor(int in1, int in2, int pwm){};
    Tb6612fngMotor(int in1, int in2, Tb6612fngLedc ledc){};
    void begin(){};

    // Velocity: -1.0..+1.0. Motor duty cycle with positive/negative direction.
    void drive(float velocity){};

    // Braking enables low resistance circuit across the motor coil (short).
    void brake(){};

    // Coasting enables high resistance circuit across the motor coil (standby).
    void coast(){};
};

class Tb6612fng
{
public:
    Tb6612fng(int standby, int in1A, int in2A, int pwmA){};
    Tb6612fng(int standby, int in1A, int in2A, Tb6612fngLedc pwmA){};
    Tb6612fng(int standby, Tb6612fngMotor *motorA){};
    Tb6612fng(int standby, int in1A, int in2A, int pwmA, int in1B, int in2B, int pwmB);
    Tb6612fng(int standby, int in1A, int in2A, Tb6612fngLedc pwmA, int in1B, int in2B, Tb6612fngLedc pwmB){};
    Tb6612fng(int standby, Tb6612fngMotor *motorA, Tb6612fngMotor *motorB){};
    ~Tb6612fng(){};
    void begin(){};
    void enable(bool value){};
    void drive(float velocity, int duration = 0, bool stop = true) { drive(velocity, velocity, duration, stop); }
    void drive(float velocityA, float velocityB, int duration = 0, bool stop = true){};
    void brake(){};
    void coast(){};
};
#endif // MOTOR_DRIVER

#ifdef WEB_SERVER
#include "WebServer.h"
#include <WebSocketsServer.h>

WebSocketsServer wsServer = WebSocketsServer(81);
bool settingsChanged = false;

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
MPU6050_6Axis_MotionApps20 mpu;
uint16_t packetSize;                // expected DMP packet size (default is 42 bytes)
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#ifdef KALMANFILTER
// Kalman Filter parameters
KalmanFilter kalmanfilter;

float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
#endif // KALMANFILTER

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
#define PID_SAMPLE_TIME 10

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

// the time we started trying to stand up
unsigned long start_prev_time = 0;

// Reverse AIN1/AIN2 and BIN1/BIN2 to reverse the direction of the motors
// Tb6612fng motors(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);
Tb6612fng motorRight(STBY, AIN1_RIGHT, AIN2_RIGHT, PWMA_RIGHT);
Tb6612fng motorLeft(STBY, BIN1_LEFT, BIN2_LEFT, PWMB_LEFT);

void IRAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

void IRAM_ATTR encoderCountRightA()
{
    encoder_count_right_a++;
}

void IRAM_ATTR encoderCountLeftA()
{
    encoder_count_left_a++;
}

#ifdef WEB_SERVER

void SendDriveMode(uint8_t num = 0)
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
        if (settingsChanged)
        {
            preferences.putFloat("Angle_kp", pidAngle.GetKp());
            preferences.putFloat("Angle_ki", pidAngle.GetKi());
            preferences.putFloat("Angle_kd", pidAngle.GetKd());
            settingsChanged = false;
        }
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
            settingsChanged = true;
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

void BalanceDriveController_Setup()
{
#ifdef WEB_SERVER
    wsServer.begin();
    wsServer.onEvent(webSocketEvent);
#endif // WEB_SERVER

#ifdef MPU6050
    mpu.initialize();
    if (!mpu.testConnection())
    {
        DB_PRINTLN("Failed to find MPU6050 chip");
        while (1)
            delay(10);
    }
    DB_PRINTLN("MPU6050 Found!");

    // configure the MPU6050 Digital Motion Processor (DMP)
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        DB_PRINTLN("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable interrupt detection
        DB_PRINTF("Enabling interrupt detection (Arduino external interrupt %d)\n", MPU_INTERRUPT);
        attachInterrupt(MPU_INTERRUPT, dmpDataReady, RISING);
        DB_PRINTLN("DMP ready! Waiting for first interrupt...");

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        DB_PRINTF("dmpGetFIFOPacketSize = %d\n", packetSize);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DB_PRINTF("DMP Initialization failed (code %d)\n", devStatus);
        while (1)
            delay(10);
    }

#ifdef KALMANFILTER
    // Kalman Filter parameters
    dt = preferences.getFloat("dt", 0.005);
    Q_angle = preferences.getFloat("Q_angle", 0.001);
    Q_gyro = preferences.getFloat("Q_gyro", 0.005);
    R_angle = preferences.getFloat("R_angle", 0.5);
    C_0 = preferences.getFloat("C_0", 1);
    K1 = preferences.getFloat("K1", 0.05);
#endif // KALMANFILTER

    // Initialize PID parameters
    pidAngle.Setpoint = 0;
    pidAngle.SetTunings(preferences.getFloat("Angle_kp", 21),
                        preferences.getFloat("Angle_ki", 140),
                        preferences.getFloat("Angle_kd", 0.8));
    pidAngle.SetSampleTime(PID_SAMPLE_TIME);
    pidAngle.SetOutputLimits(-255, 255);
    pidAngle.SetMode(MANUAL);

#endif // MPU6050

    // setup the motors and attach the rotary encoder counters
    // motors.begin();
    motorLeft.begin();
    motorRight.begin();

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
    pwm_left = -pidAngle.Output;  // - speed_control_output - rotation_control_output;
    pwm_right = -pidAngle.Output; // - speed_control_output + rotation_control_output;
    //    pwm_left = constrain(pwm_left, -255, 255);
    //    pwm_right = constrain(pwm_right, -255, 255);

    motorLeft.drive(pwm_left / 255.0);
    motorRight.drive(pwm_right / 255.0);

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

#ifdef MPU6050
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;                    // [w, x, y, z]         quaternion container
    VectorFloat gravity;             // [x, y, z]            gravity vector
    static float ypr[3] = {0, 0, 0}; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // If we have new motion processor data, update pidAngle.Input
    fifoCount = mpu.getFIFOCount();
    if (mpuInterrupt || fifoCount >= packetSize)
    {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        if ((mpuIntStatus & 0x10) || fifoCount >= 1024)
        {
            mpu.resetFIFO();
            DB_PRINTLN("FIFO overflow!");
        }
        else if (mpuIntStatus & 0x02)
        {
            // wait until we have a complete packet
            while (fifoCount < packetSize)
                fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);      // get value for q
            mpu.dmpGetGravity(&gravity, &q);           // get value for gravity
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr

            // given the orientation of the MPU6050, the 'roll' is actually the 'pitch' value we need
            pidAngle.Input = ypr[2] * 180 / M_PI;      // convert output to degrees
        }
    }
#endif // MPU6050

    // only run every PID_SAMPLE_TIME ms
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if ((currentTime - lastTime) < PID_SAMPLE_TIME)
        return;

#ifdef MPU6050

#ifdef KALMANFILTER
    // read the IMU and compute use a Kalman Filter to compute a filtered angle in degrees for the robot
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
#endif // KALMANFILTER

    // calculate the correction needed to balance
    pidAngle.Compute();

    switch (drive_mode)
    {
    case MODE_PARKED:
        break;

    case MODE_STANDING_UP:
        // Detect if robot is balanced enough that we can transition to driving.
        // Ensure we are beyond BALANCE_ANGLE_MAX enough that we don't transition to DRIVE then detect a fall.
        if (abs(pidAngle.Input) < BALANCE_ANGLE_MAX / 2)
        {
            BalanceDriveController_SetMode(MODE_DRIVE);
            break;
        }

        // if it's been 1000 ms since we started to stand
        if (millis() - start_prev_time > 1000)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
        }
        break;

    case MODE_DRIVE:
        // If we exeed BALANCE_ANGLE_MAX in the positive direction, we will land on the leg and transition to MODE_PARKED
        if (pidAngle.Input > BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
            break;
        }

        // If we exceed BALANCE_ANGLE_MAX in the negative direction, we have FALLEN
        if (pidAngle.Input < -BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_FALLEN);
            break;
        }

        UpdateMotors();
        break;

    case MODE_PARKING:
        // If we exeed BALANCE_ANGLE_MAX in the positive direction, we will land on the leg and transition to MODE_PARKED
        if (pidAngle.Input > BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
            break;
        }

        // if it's been 500 ms since we started to park
        if (millis() - start_prev_time > 500)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
        }
        break;

    case MODE_FALLEN:
        // If we exeed BALANCE_ANGLE_MAX in the positive direction, we will land on the leg and transition to MODE_PARKED
        if (pidAngle.Input > BALANCE_ANGLE_MAX)
        {
            BalanceDriveController_SetMode(MODE_PARKED);
            break;
        }
        break;

    case MODE_CALIBRATION:
        break;
    }
#ifdef WEB_SERVER
    static uint8_t k = 0;
    if (k == plot.prescaler)
    {
        k = 0;

        if (wsServer.connectedClients(0) > 0 && plot.enable)
        {
            float plotData[15] = {0};

            plotData[0] = pidAngle.Input;
            plotData[1] = pidAngle.Output;
#ifdef KALMANFILTER
            plotData[2] = kalmanfilter.angle;
            plotData[3] = kalmanfilter.Gyro_x;
#endif // KALMANFILTER
            plotData[4] = ypr[0] * 180 / M_PI;
            plotData[5] = ypr[2] * 180 / M_PI; // pitch and roll are reversed due to how the MPU6050
            plotData[6] = ypr[1] * 180 / M_PI; // is mounted on the breadboard
#ifdef KALMANFILTER
            plotData[7] = gx;
            plotData[8] = gy;
            plotData[9] = gz;
#endif
            plotData[10] = encoder_count_left_a;
            plotData[11] = encoder_count_right_a;
            plotData[12] = pwm_left;
            plotData[13] = pwm_right;
            plotData[14] = currentTime - lastTime;
            wsServer.sendBIN(0, (uint8_t *)plotData, sizeof(plotData));
        }
    }
    k++;
#endif

#ifdef MOTOR_SERIAL_PLOTTER
    // serial plotter friendly format
    SerialPlotterOutput = true;
    DB_PRINT("correctedAngle:");
#ifdef KALMANFILTER
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
#endif // KALMANFILTER
    DB_PRINT("encoderCountLeft:");
    DB_PRINT(encoder_count_left);
    DB_PRINT(",");
    DB_PRINT("encoderCountRight:");
    DB_PRINT(encoder_count_right);
    DB_PRINT(",");
    DB_PRINT("motorLeft:");
    DB_PRINT(pwm_left / 255.0);
    DB_PRINT(",");
    DB_PRINT("motorRight:");
    DB_PRINT(pwm_right / 255.0);
    DB_PRINT(",");
    DB_PRINT("ElapsedTime:");
    DB_PRINT(currentTime - lastTime);
    DB_PRINT(",");
#endif // MOTOR_SERIAL_PLOTTER

    // reset the encoder counts now that we've had a chance to use and report them
    encoder_count_left_a = 0;
    encoder_count_right_a = 0;

#endif // MPU6050

    // update our time tracking
    lastTime = currentTime;
}

void BalanceDriveController_SetMode(DriveMode newDriveMode)
{
    if (newDriveMode == drive_mode)
        return;

    // prevent invalid transitions while fallen
    if ((drive_mode == MODE_FALLEN) && newDriveMode != MODE_PARKED)
        return;

    switch (newDriveMode)
    {
    case MODE_PARKED: // robot is parked on its leg so it can later stand
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_PARKED");
        pidAngle.SetMode(MANUAL); // we don't want the PID accumulating error when we aren't trying to balance
        pwm_left = pwm_right = 0;
        motorLeft.brake();
        motorRight.brake();
        break;

    case MODE_STANDING_UP: // robot is in the process of standing
        // if we aren't parked, ignore the request to stand up
        if (drive_mode != MODE_PARKED)
            return;

        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_STANDING_UP");
        pidAngle.SetMode(AUTOMATIC); // we need the PID running so we can transition smoothly to driving

        // go full speed forward so we can stand up
        pwm_left = pwm_right = 255;
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
        start_prev_time = millis();
        break;

    case MODE_PARKING: // robot is in the process of parking
        // if we aren't driving, ignore the request to park
        if (drive_mode != MODE_DRIVE)
            return;

        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_PARKING");
        pidAngle.SetMode(MANUAL); // we don't want the PID accumulating error when we aren't trying to balance

        // backup so we rest on the foot then hit the brakes
        pwm_left -= 128;
        pwm_right -= 128;
        motorLeft.drive(pwm_left / 255.0);
        motorRight.drive(pwm_right / 255.0);
        start_prev_time = millis();
        break;

    case MODE_DRIVE: // robot is balancing and able to be driven
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_DRIVE");
        pidAngle.SetMode(AUTOMATIC); // we only want the PID running when we're trying to balance
        break;

    case MODE_FALLEN: // robot has fallen and can't get up
        DB_PRINTLN("BalanceDriveController_SetMode: transition to MODE_FALLEN");
        pidAngle.SetMode(MANUAL); // we don't want the PID accumulating error when we aren't trying to balance
        pwm_left = pwm_right = 0;
        motorLeft.brake();
        motorRight.brake();
        break;

    case MODE_CALIBRATION: // calculate a new calibration and store it in the MPU6050
        // tell the web page we're calibrating then turn off pidAngle and motors just to be sure we aren't moving
        drive_mode = newDriveMode;
        SendDriveMode();
        pidAngle.SetMode(MANUAL); // we don't want the PID accumulating error when we aren't trying to balance
        pwm_left = pwm_right = 0;
        motorLeft.brake();
        motorRight.brake();
        // now actually run the calibration then switch us to MODE_PARKED
        DB_PRINTLN("PID tuning - each dot = 100 readings");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        DB_PRINTLN(" Calibration complete and stored in MPU6050");
        newDriveMode = MODE_PARKED;
        break;
    }

    drive_mode = newDriveMode;
#ifdef WEB_SERVER
    SendDriveMode();
#endif
}

void BalanceDriveController_SetVelocity(float newSpeed, float newSteer)
{
    speed = newSpeed;
    steer = newSteer;
}
