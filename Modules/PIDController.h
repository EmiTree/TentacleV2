#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float update(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue);
    void reset();

    void setKp(float newKp);
    void setKi(float newKi);
    void setKd(float newKd);

    float getKp();
    float getKi();
    float getKd();

private:
    float kp;
    float ki;
    float kd;

    float previousError;
    float integral;
};