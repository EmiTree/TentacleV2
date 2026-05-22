#pragma once

#include "pico/stdlib.h"

class ServoActuator {
public:
    ServoActuator(int servo1Pin, int servo2Pin, int servo3Pin, int servo4Pin);

    void begin();
    void update();

    void setSpeed(float speedPercent);
    void setServo1Speed(float speedPercent);
    void setServo2Speed(float speedPercent);
    void setServo3Speed(float speedPercent);
    void setServo4Speed(float speedPercent);
    void stop();

    void moveToAngle(float targetRealAngle);
    void moveByAngle(float realAngleDifference);
    void zeroAngleHere();

    void printStatus();
    void printHelp();

    void setMinAngle(float minAngle);
    void setMaxAngle(float maxAngle);

private:
    int servoPins[4];
    float servoSpeeds[4];
    float estimatedAngles[4];

    bool timedMoveRunning;
    float timedMoveTargetAngle;
    absolute_time_t timedMoveEndTime;

    absolute_time_t lastAngleUpdateTime;

    float clampFloat(float value, float low, float high);
    int clampInt(int value, int low, int high);

    void setupServoPwm(int pin);
    int speedToPulseUs(float speedPercent);
    void setServoSpeedPin(int servoIndex, float speedPercent);
    void setOneServoSpeedByIndex(int servoIndex, float speedPercent);
    void setAllServoSpeeds(float speedPercent);
    void updateEstimatedAnglesAndFailsafes();
    void startTimedMove(float targetAngle, float angleDifference);
};