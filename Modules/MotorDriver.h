#pragma once

class MotorDriver {
public:
    MotorDriver(int pinP1, int pinP2, int pinQ1, int pinQ2);

    void begin();
    void drive(float pwmA, float pwmB);
    void stop();

private:
    int motorPinP1;
    int motorPinP2;
    int motorPinQ1;
    int motorPinQ2;

    static constexpr int maximumLevel = 1000;

    void setupMotorPWM(int pin);
    float constrainValue(float value, float minVal, float maxVal);
};