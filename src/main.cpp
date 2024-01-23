#define DEBUG 1
#define XBOX_CONTROLLER
//#define DEBUG_XBOX_CONTROLLER
//#define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#define MPU6050
#define MPU6050_SERIAL_PLOTTER
#include <Arduino.h>

#ifdef MPU6050
#include <Adafruit_MPU6050.h>
#include <KalmanFilter.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;
KalmanFilter kalmanfilter;
float kalmanfilter_angle;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
#endif

#ifdef XBOX_CONTROLLER
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// only bind to my xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("9c:aa:1b:f2:66:3d");

// bind to any xbox controller
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
#endif

#ifdef BALANCE_CAR
#include "BalanceCar.h"
#endif
#ifdef CHECK_VOLTAGE
#include "voltage.h"
#endif

#ifdef BALANCE_CAR
unsigned long start_prev_time = 0;
boolean carInitialize_en = true;
#endif

#ifdef BALANCE_CAR
void setMotionState()
{
  switch (motion_mode)
  {
  case FORWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = 40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case BACKWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = -80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = -40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case TURNLEFT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case BLUETOOTH:
      setting_turn_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = 80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    }
    break;
  case TURNRIGHT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case BLUETOOTH:
      setting_turn_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = -80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    }
    break;
  case STANDBY:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    break;
  case STOP:
    if (millis() - start_prev_time > 1000)
    {
      function_mode = IDLE;
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        motion_mode = STANDBY;
      }
    }
    break;
  case START:
    if (millis() - start_prev_time > 2000)
    {
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        car_speed_integeral = 0;
        setting_car_speed = 0;
        motion_mode = STANDBY;
      }
      else
      {
        motion_mode = STOP;
        carStop();
      }
    }
    break;
  default:
    break;
  }
}

void keyEventHandle()
{
  if (key_value != '\0')
  {
    key_flag = key_value;

    switch (key_value)
    {
    case 's':
      motion_mode = STANDBY;
      break;
    case 'f':
      motion_mode = FORWARD;
      break;
    case 'b':
      motion_mode = BACKWARD;
      break;
    case 'l':
      motion_mode = TURNLEFT;
      break;
    case 'i':
      motion_mode = TURNRIGHT;
      break;
    case '1':
      function_mode = FOLLOW;
      follow_flag = 0;
      follow_prev_time = millis();
      break;
    case '2':
      function_mode = OBSTACLE;
      obstacle_avoidance_flag = 0;
      obstacle_avoidance_prev_time = millis();
      break;

    case '3':
    rgb_loop:
      break;
    case '4':
      function_mode = IDLE;
      motion_mode = STOP;
      carBack(110);
      delay((kalmanfilter_angle - 30) * (kalmanfilter_angle - 30) / 8);
      carStop();
      start_prev_time = millis();
      rgb.brightRedColor();
      break;
    case '5':
      if (millis() - start_prev_time > 500 && kalmanfilter_angle >= balance_angle_min)
      {
        start_prev_time = millis();
        motion_mode = START;
      }
      motion_mode = START;
      break;
    case '6':
      rgb.brightness = 50;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '7':
      rgb.brightRedColor();
      rgb.brightness -= 25;
      if (rgb.brightness <= 0)
      {
        rgb.brightness = 0;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '8':
      rgb.brightRedColor();
      rgb.brightness += 25;
      if (rgb.brightness >= 255)
      {
        rgb.brightness = 255;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '9':
      rgb.brightness = 0;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '0':
      function_mode = FOLLOW2;
      follow_flag = 0;
      follow_prev_time = millis();
      break;
    case '*':
      break;
    case '#':
      break;
    default:
      break;
    }
    if (key_flag == key_value)
    {
      key_value = '\0';
    }
  }
}
#endif

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
#endif

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

#ifdef XBOX_CONTROLLER
#ifdef DEBUG_XBOX_CONTROLLER
  Serial.println("Starting NimBLE Client");
#endif
  xboxController.begin();
#endif

#ifdef CHECK_VOLTAGE
  voltageInit(); // Check that the voltage of the battery is high enough
#endif

#ifdef BALANCE_CAR
  start_prev_time = millis();
  carInitialize();
#endif
}

void loop()
{
#ifdef MPU6050
  /* Get a new normalized sensor event */
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  kalmanfilter.Angle(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  
#ifndef MPU6050_SERIAL_PLOTTER
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);
#else
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
  delay(10);
#endif
#endif

#ifdef XBOX_CONTROLLER
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
#ifdef DEBUG_XBOX_CONTROLLER
      Serial.println("waiting for first notification");
#endif
    }
    else
    {
#ifdef DEBUG_XBOX_CONTROLLER
      Serial.println("Address: " + xboxController.buildDeviceAddressStr());
      Serial.print(xboxController.xboxNotif.toString());
#endif
      unsigned long receivedAt = xboxController.getReceiveNotificationAt();
      uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
#ifdef DEBUG_XBOX_CONTROLLER
      Serial.print("joyLHori rate: ");
      Serial.println((float)xboxController.xboxNotif.joyLHori / joystickMax);
      Serial.print("joyLVert rate: ");
      Serial.println((float)xboxController.xboxNotif.joyLVert / joystickMax);
      Serial.println("battery " + String(xboxController.battery) + "%");
      Serial.println("received at " + String(receivedAt));
#endif
    }
  }
  else
  {
#ifdef DEBUG_XBOX_CONTROLLER
    Serial.println("not connected");
#endif
    if (xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
#endif

#ifdef BALANCE_CAR
  keyEventHandle();
#endif

#ifdef CHECK_VOLTAGE
  voltageMeasure();
#endif

#ifdef BALANCE_CAR
  setMotionState();

#ifdef DEBUG
  static unsigned long print_time;
  if (millis() - print_time > 100)
  {
    print_time = millis();
    Serial.println(kalmanfilter.angle);
  }
#endif

  static unsigned long start_time;
  if (millis() - start_time < 10)
  {
    function_mode = IDLE;
    motion_mode = STOP;
    carStop();
  }
  if (millis() - start_time == 2000) // Enter the pendulum, the car balances...
  {
    key_value = '5';
  }
#endif
}
