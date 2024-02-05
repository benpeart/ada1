#include "pid.h"

PID::PID(float Kp, float Ki, float Kd)
{
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    lastInput = 0.0;
    integral = 0.0;
}

float PID::PID_Update(float setpoint, float input)
{
    // Calculate error
    float error = setpoint - input;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    integral += error;
    float Iout = Ki * integral;

    // Derivative term
    float derivative = input - lastInput;
    float Dout = Kd * derivative;

    // Remember last input for next time
    lastInput = input;

    // return total output
    return Pout + Iout - Dout;
}
