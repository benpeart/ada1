#include "conditional.h"
#include <Arduino.h>
#include "debug.h"
#include <Preferences.h> // for storing settings in the ESP32 EEPROM

#ifdef WEB_SERVER
#include <AsyncTCP.h>
#include <ESPAsyncWiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>

char robotName[63] = "ada";
const char *http_username = "admin";
const char *http_password = "admin";
AsyncWebServer httpServer(80);
DNSServer dns;
#endif // WEB_SERVER

// -- EEPROM
Preferences preferences;
#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define PREF_NAMESPACE "pref"
#define PREF_KEY_VERSION "ver"

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

#include "BalanceDriveController.h"

void setup()
{
  Serial.begin(115200);

  preferences.begin(PREF_NAMESPACE, false); // false = RW-mode

  // Init EEPROM, if not done before
  if (preferences.getUInt(PREF_KEY_VERSION, 0) != PREF_VERSION)
  {
    preferences.clear(); // Remove all preferences under the opened namespace
    preferences.putUInt(PREF_KEY_VERSION, PREF_VERSION);
    DB_PRINTF("EEPROM init complete, all preferences deleted, new pref_version: %d\n", PREF_VERSION);
  }

#ifdef WEB_SERVER
  // SPIFFS setup
  if (!SPIFFS.begin(false))
  {
    DB_PRINTLN("SPIFFS mount failed");
    return;
  }
  else
  {
    DB_PRINTLN("SPIFFS mount success");
  }

  // Read robot name
  preferences.getBytes("robot_name", robotName, sizeof(robotName));
  WiFi.setHostname(robotName);

  // WiFiManager local intialization. Once its business is done, there is no need to keep it around
  AsyncWiFiManager wifiManager(&httpServer, &dns);
  // wifiManager.resetSettings();

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(robotName);
  if (!(WiFi.waitForConnectResult() != WL_CONNECTED))
  {
    DB_PRINT("Connected to WiFi with IP address: ");
    DB_PRINTLN(WiFi.localIP());
  }
  else
  {
    DB_PRINTLN("Could not connect to known WiFi network");
  }

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                {
    DB_PRINTLN("Loading index.html");
    request->send(SPIFFS, "/index.html"); });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request)
                        { request->send(404, "text/plain", "FileNotFound"); });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS, http_username, http_password));
  ElegantOTA.begin(&httpServer); // Start ElegantOTA
  httpServer.begin();
#endif // WEB_SERVER

#ifdef XBOX_CONTROLLER
#ifdef DEBUG_XBOX_CONTROLLER
  DB_PRINTLN("Starting NimBLE Client");
#endif
  xboxController.begin();
#endif

  BalanceDriveController_Setup(preferences);
}

void loop()
{
  ElegantOTA.loop();

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
      // normalize the controller input to the range of 0 to 1 then scale
      float car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
      float car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig);

      // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
      // the steering is based off the left horizontal joystick
      float speed = car_speed_forward - car_speed_reverse;
      float steer = (float)xboxController.xboxNotif.joyLHori / XboxControllerNotificationParser::maxJoy;
      BalanceDriveController_SetVelocity(speed, steer);

      // The "A" button will tell us to stand up and start to balance
      if (xboxController.xboxNotif.btnA)
        BalanceDriveController_SetMode(MODE_STANDING_UP);

      // The 'B' button will tell us to tip over onto the leg and stop balancing
      if (xboxController.xboxNotif.btnB)
        BalanceDriveController_SetMode(MODE_PARKING);

      // The 'Start' button will tell us to start the calibration process
      if (xboxController.xboxNotif.btnStart)
        BalanceDriveController_SetMode(MODE_CALIBRATION);

#ifdef XBOX_SERIAL_PLOTTER
      SerialPlotterOutput = true;
      DB_PRINT("speed:");
      DB_PRINT(speed);
      DB_PRINT(",");
      DB_PRINT("steer:");
      DB_PRINT(steer);
      DB_PRINT(",");
#endif // XBOX_SERIAL_PLOTTER
    }
  }
  else
  {
#ifdef DEBUG_XBOX_CONTROLLER
    DB_PRINTLN("not connected");
#endif
    // To prevent rebooting when a controller turns on but doesn't connect we could
    // start with an empty address and reboot if failed connections. Once connected,
    // save the address in preferences (with a WebUI to clear it) and use that without
    // the reboot logic on future boots.
    if (xboxController.getCountFailedConnection() > 3)
    {
      ESP.restart();
    }
  }
#endif

  BalanceDriveController_Loop();

#ifdef DEBUG
  if (SerialPlotterOutput)
  {
    DB_PRINTLN();
    SerialPlotterOutput = false;
  }
#endif
}
