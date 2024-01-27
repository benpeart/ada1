/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MPU6050.h>
#include <Arduino_JSON.h>
#include <SPIFFS.h>
#include "WebServer.h"
#include "debug.h"

// Replace with your network credentials
const char *ssid = "IOT";
const char *password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
    DB_PRINTLN("An error has occurred while mounting SPIFFS");
  }
  DB_PRINTLN("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  DB_PRINTLN("");
  DB_PRINT("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    DB_PRINT(".");
    delay(1000);
  }
  DB_PRINTLN("");
  DB_PRINTLN(WiFi.localIP());
}

String getGyroReadings(MPU6050 &mpu)
{
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings(MPU6050 &mpu)
{
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify(readings);
  return accString;
}

String getTemperature(MPU6050 &mpu)
{
  temperature = mpu.getTemperature();
  return String(temperature);
}

void webserver_setup()
{
  initWiFi();
  initSPIFFS();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    gyroX=0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    gyroY=0;
    request->send(200, "text/plain", "OK"); });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    gyroZ=0;
    request->send(200, "text/plain", "OK"); });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client)
                   {
    if(client->lastId()){
      DB_PRINTF("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000); });
  server.addHandler(&events);

  server.begin();
}

void webserver_loop(MPU6050_Base &mpu)
{
  if ((millis() - lastTime) > gyroDelay)
  {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings(mpu).c_str(), "gyro_readings", millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay)
  {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings(mpu).c_str(), "accelerometer_readings", millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay)
  {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature(mpu).c_str(), "temperature_reading", millis());
    lastTimeTemperature = millis();
  }
}
