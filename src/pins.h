// Pins for all motor input and outputs
#define PWMB_LEFT 33
#define BIN1_LEFT 26
#define BIN2_LEFT 25
#define PWMA_RIGHT 13
#define AIN1_RIGHT 14
#define AIN2_RIGHT 12
#define STBY 27
#define ENCODER_LEFT_A_PIN 5
#define ENCODER_RIGHT_A_PIN 4

#if 0 // alternate pins that should work if issues with above
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA T0
#define PWMB T2
#define STBY 9
#endif
