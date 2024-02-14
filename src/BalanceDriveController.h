#ifndef BalanceDriveController_h
#define BalanceDriveController_h

#include <Preferences.h> // for storing settings in the ESP32 EEPROM

#define SPEED_FORWARD_MAX   80
#define SPEED_REVERSE_MAX   80
extern int setting_car_speed;

#define TURN_LEFT_MAX   80
#define TURN_RIGHT_MAX  -80
extern int setting_turn_speed;

void BalanceDriveController_Setup(Preferences &preferences);

#ifdef WEB_SERVER
#include <WebSocketsServer.h>
void BalanceDriveController_Loop(WebSocketsServer &wsServer);
#else
void BalanceDriveController_Loop();
#endif

#endif // BalanceDriveController_h
