#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float update(float setpoint, float measuredValue, float dt);
    void reset();

private:
    float kp;
    float ki;
    float kd;

    float previousError;
    float integral;
};