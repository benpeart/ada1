#ifndef BalanceDriveController_h
#define BalanceDriveController_h

#define SPEED_FORWARD_MAX   80
#define SPEED_REVERSE_MAX   80
extern int setting_car_speed;

#define TURN_LEFT_MAX   80
#define TURN_RIGHT_MAX  -80
extern int setting_turn_speed;

void BalanceDriveController_Setup();
void BalanceDriveController_Loop();

#endif // BalanceDriveController_h
