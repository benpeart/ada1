#include "conditional.h"
#include <Arduino.h>
#include "debug.h"
#include <Preferences.h> // for storing settings in the ESP32 EEPROM

#ifdef WEB_SERVER
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>

char robotName[63] = "ada";
const char *http_username = "admin";
const char *http_password = "admin";
AsyncWebServer httpServer(80);
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

#ifdef BALANCE_DRIVE_CONTROLLER
#include "BalanceDriveController.h"
#endif

void setup()
{
  Serial.begin(230400);

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

  // Connect to Wifi and setup OTA if known Wifi network cannot be found
  boolean wifiConnected = 0;
  //  if (preferences.getUInt("wifi_mode", 0) == 1)
  {
    char ssid[63] = "IOT";
    char password[63] = "";
    preferences.getBytes("wifi_ssid", ssid, sizeof(ssid));
    preferences.getBytes("wifi_key", password, sizeof(password));

    DB_PRINTF("Connecting to '%s'\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (!(WiFi.waitForConnectResult() != WL_CONNECTED))
    {
      DB_PRINT("Connected to WiFi with IP address: ");
      DB_PRINTLN(WiFi.localIP());
      wifiConnected = 1;
    }
    else
    {
      DB_PRINTLN("Could not connect to known WiFi network");
    }
  }
  if (!wifiConnected)
  {
    DB_PRINTLN("Starting AP...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(robotName, "turboturbo");
    DB_PRINTF("AP named '%s' started, IP address: %s\n", WiFi.softAPSSID(), WiFi.softAPIP());
  }

  // setup for OTA flash updates
  ArduinoOTA.setHostname(robotName);
  ArduinoOTA
      .onStart([]()
               {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    DB_PRINTLN("Start updating " + type); })
      .onEnd([]()
             { DB_PRINTLN("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { DB_PRINTF("Progress: %u%%\r\n", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
    DB_PRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DB_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DB_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DB_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DB_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DB_PRINTLN("End Failed"); });

  ArduinoOTA.begin();

  // Start DNS server
  if (MDNS.begin(robotName))
  {
    DB_PRINTF("MDNS responder started, name: %s\n", robotName);
  }
  else
  {
    DB_PRINTLN("Could not start MDNS responder");
  }

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                {
    DB_PRINTLN("Loading index.htm");
    request->send(SPIFFS, "/index.htm"); });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request)
                        { request->send(404, "text/plain", "FileNotFound"); });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS, http_username, http_password));
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
#endif // WEB_SERVER

#ifdef XBOX_CONTROLLER
#ifdef DEBUG_XBOX_CONTROLLER
  DB_PRINTLN("Starting NimBLE Client");
#endif
  xboxController.begin();
#endif

#ifdef BALANCE_DRIVE_CONTROLLER
  BalanceDriveController_Setup(preferences);
#endif
}

void loop()
{
#ifdef WEB_SERVER
  ArduinoOTA.handle();
#endif

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
#endif // BALANCE_DRIVE_CONTROLLER

#ifdef DEBUG
  if (SerialPlotterOutput)
  {
    DB_PRINTLN();
    SerialPlotterOutput = false;
  }
#endif
}
