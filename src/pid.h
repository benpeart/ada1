#ifndef PID_h
#define PID_h

class PID
{
public:
    PID(float Kp, float Ki, float Kd);
    float PID_Update(float setpoint, float input);

private:
    float Kp = 1.0;
    float Ki = 0.5;
    float Kd = 0.1;
    float lastInput = 0.0;
    float integral = 0.0;
};

#endif // PID_h