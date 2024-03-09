#ifndef BalanceDriveController_h
#define BalanceDriveController_h

// Standard Arduino setup/loop functions. Loop expects to be called on every call to void loop();
void BalanceDriveController_Setup();
void BalanceDriveController_Loop();

// Handle state transitions and ensure only valid combinations are made.
typedef enum
{
    MODE_PARKED,      // robot is parked on its leg so it can later stand
    MODE_STANDING_UP, // robot is in the process of standing
    MODE_PARKING,     // robot is in the process of parking
    MODE_DRIVE,       // robot is balancing and able to be driven
    MODE_FALLEN,      // robot has fallen and can't get up
    MODE_CALIBRATION  // calculate a new angle_zero calibration
} DriveMode;
void BalanceDriveController_SetMode(DriveMode newDriveMode);

// Keeping this simple for now.
// speed -1 means full speed reverse, 0 means balance in place, +1 means full speed forward.
// steer -1 means spin in place left, 0 means don't turn, 1 means spin in place right.
//
// ROS2 will want a geometry message that contains:
//      message.linear.x = 0.5;  // Move forward at 0.5 m/s
//      message.angular.z = 0.1; // Turn at 0.1 rad/s
void BalanceDriveController_SetVelocity(float speed, float steer);

#endif // BalanceDriveController_h
