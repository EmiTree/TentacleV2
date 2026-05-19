#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    previousError = 0.0f;
    integral = 0.0f;
}

float PIDController::update(float setpoint, float measuredValue, float dt) {
    float error = setpoint - measuredValue;

    integral += error * dt;

    float derivative = 0.0f;

    if (dt > 0.0f) {
        derivative = (error - previousError) / dt;
    }

    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
    previousError = 0.0f;
    integral = 0.0f;
}