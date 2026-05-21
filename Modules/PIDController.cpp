#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    previousError = 0.0f;
    integral = 0.0f;
}

float PIDController::update(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue) {
    float error = setpoint - measuredValue;

    integral += error * dt;

    float derivative = 0.0f;

    if (dt > 0.0f) {
        derivative = (error - previousError) / dt;
    }

    pValue = kp * error;
    iValue = ki * integral;
    dValue = kd * derivative;

    previousError = error;

    return pValue + iValue + dValue;
}

void PIDController::reset() {
    previousError = 0.0f;
    integral = 0.0f;
}

void PIDController::setKp(float newKp) {
    kp = newKp;
}

void PIDController::setKi(float newKi) {
    ki = newKi;
}

void PIDController::setKd(float newKd) {
    kd = newKd;
}

float PIDController::getKp() {
    return kp;
}

float PIDController::getKi() {
    return ki;
}

float PIDController::getKd() {
    return kd;
}