#define DEBUG 1
#define XBOX_CONTROLLER
// #define DEBUG_XBOX_CONTROLLER
// #define XBOX_SERIAL_PLOTTER
// #define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#define BALANCE_DRIVE_CONTROLLER

#include <Arduino.h>
#include "debug.h"

#ifdef DEBUG
boolean SerialPlotterOutput = false;
#endif

#ifdef XBOX_CONTROLLER
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// only bind to my xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("9c:aa:1b:f2:66:3d");

// bind to any xbox controller
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
#endif

#ifdef BALANCE_DRIVE_CONTROLLER
#include "BalanceDriveController.h"
#endif

#ifdef CHECK_VOLTAGE
#include "voltage.h"
#endif

#ifdef BALANCE_CAR
#include "BalanceCar.h"

unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

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
  Serial.begin(230400);
#ifdef DEBUG
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
#endif

#ifdef XBOX_CONTROLLER
#ifdef DEBUG_XBOX_CONTROLLER
  DB_PRINTLN("Starting NimBLE Client");
#endif
  xboxController.begin();
#endif

#ifdef CHECK_VOLTAGE
  voltageInit(); // Check that the voltage of the battery is high enough
#endif

#ifdef BALANCE_DRIVE_CONTROLLER
  BalanceDriveController_Setup();
#endif

#ifdef BALANCE_CAR
  start_prev_time = millis();
  carInitialize();
#endif
}

void loop()
{
#ifdef XBOX_CONTROLLER
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
#ifdef DEBUG_XBOX_CONTROLLER
      DB_PRINTLN("waiting for first notification");
#endif
    }
    else
    {
#ifdef DEBUG_XBOX_CONTROLLER
      static int first = 1;
      if (first)
      {
        first = 0;
        DB_PRINTLN("Address: " + xboxController.buildDeviceAddressStr());
        DB_PRINT(xboxController.xboxNotif.toString());
      }
#endif
      // normalize the controller input to the range of 0 to 1 then scale and then scale and shift it to the max speed range
      int car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig) * SPEED_FORWARD_MAX - SPEED_FORWARD_MAX;
      int car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig) * SPEED_REVERSE_MAX - SPEED_REVERSE_MAX;

      // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
      setting_car_speed = car_speed_forward - car_speed_reverse;

      // the turn speed is based off the left horizontal joystick scaled to the MAX turn speed
      setting_turn_speed = ((float)xboxController.xboxNotif.joyLHori / XboxControllerNotificationParser::maxJoy) * (TURN_LEFT_MAX - TURN_RIGHT_MAX) - TURN_LEFT_MAX;
#ifdef XBOX_SERIAL_PLOTTER
      SerialPlotterOutput = true;
      DB_PRINT("setting_turn_speed:");
      DB_PRINT((float)setting_turn_speed);
      DB_PRINT(",");
      DB_PRINT("setting_car_speed:");
      DB_PRINT((float)setting_car_speed);
      DB_PRINT(",");
#endif // XBOX_SERIAL_PLOTTER
    }
  }
  else
  {
#ifdef DEBUG_XBOX_CONTROLLER
    DB_PRINTLN("not connected");
#endif
#if 0
    // This does not appear to be necessary when specifying a controller address.
    // To prevent rebooting when a controller turns on but doesn't connect we could
    // start with an empty address and reboot if failed connections. Once connected, 
    // save the address in preferences (with a WebUI to clear it) and use that without
    // the reboot logic on future boots.
    if (xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
#endif
  }
#endif

#ifdef BALANCE_DRIVE_CONTROLLER
  BalanceDriveController_Loop();
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
    DB_PRINTLN(kalmanfilter.angle);
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
#ifdef DEBUG
  if (SerialPlotterOutput)
  {
    DB_PRINTLN();
    SerialPlotterOutput = false;
  }
#endif
}
