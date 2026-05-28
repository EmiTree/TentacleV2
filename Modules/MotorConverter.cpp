#include "MotorConverter.h"

#include <stdio.h>
#include <math.h>

MotorConverter::MotorConverter(float pidOutputLimit, float maxPwm, float motorStartPwm) {
    this->pidOutputLimit = pidOutputLimit;
    this->maxPwm = maxPwm;
    this->motorStartPwm = motorStartPwm;

    /*
        Default extra behavior.

        Deadband is on by default to prevent tiny PID outputs from immediately
        giving the motors start power.

        Response curve is off by default because 1.0 is the normal linear
        behavior.
    */
    deadband = 0.03f;
    responseCurve = 1.0f;

    deadbandEnabled = true;
    responseCurveEnabled = false;
}

float MotorConverter::clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

MotorCommand MotorConverter::convert(float pidOutput) {
    MotorCommand command;

    command.pwmA = 0.0f;
    command.pwmB = 0.0f;
    command.motorOutput = 0.0f;
    command.normalizedOutput = 0.0f;
    command.scaledOutput = 0.0f;

    float safePidOutputLimit = clampFloat(pidOutputLimit, 1.0f, 1000.0f);
    float safeMaxPwm = clampFloat(maxPwm, 0.0f, 100.0f);
    float safeMotorStartPwm = clampFloat(motorStartPwm, 0.0f, safeMaxPwm);

    float availablePwmRange = safeMaxPwm - safeMotorStartPwm;

    if (availablePwmRange <= 0.0f) {
        return command;
    }

    /*
        Normalize PID output.

        Example:
            pidOutput = 30
            pidOutputLimit = 60

            normalized = 0.5
    */
    float normalized = pidOutput / safePidOutputLimit;
    normalized = clampFloat(normalized, -1.0f, 1.0f);

    command.normalizedOutput = normalized;

    float direction = 1.0f;

    if (normalized < 0.0f) {
        direction = -1.0f;
    }

    float effort = normalized;

    if (effort < 0.0f) {
        effort = -effort;
    }

    /*
        Deadband.

        If effort is tiny, the motors stay off.
        This avoids motor twitching around the setpoint.

        If deadband is disabled, effort passes through normally.
    */
    if (deadbandEnabled) {
        float safeDeadband = clampFloat(deadband, 0.0f, 0.95f);

        if (effort < safeDeadband) {
            return command;
        }

        /*
            Re-scale after the deadband.

            Example with deadband 0.1:
                effort 0.1 becomes 0.0
                effort 1.0 becomes 1.0

            This makes the usable range smooth again.
        */
        effort = (effort - safeDeadband) / (1.0f - safeDeadband);
    }

    effort = clampFloat(effort, 0.0f, 1.0f);

    /*
        Response curve.

        responseCurve = 1.0 means normal linear behavior.
        responseCurve = 1.5 or 2.0 makes small corrections gentler.

        If responseCurveEnabled is false, this stays linear.
    */
    if (responseCurveEnabled) {
        float safeCurve = clampFloat(responseCurve, 0.1f, 5.0f);
        effort = powf(effort, safeCurve);
    }

    command.scaledOutput = effort;

    /*
        Convert the 0..1 effort into PWM.

        motorStartPwm is only added after the deadband. So tiny PID outputs do
        not immediately jump to motorStartPwm.
    */
    float pwm = safeMotorStartPwm + effort * availablePwmRange;
    pwm = clampFloat(pwm, 0.0f, safeMaxPwm);

    command.motorOutput = direction * pwm;

    if (direction > 0.0f) {
        command.pwmA = pwm;
        command.pwmB = 0.0f;
    } else {
        command.pwmA = 0.0f;
        command.pwmB = pwm;
    }

    return command;
}

void MotorConverter::setPidOutputLimit(float value) {
    pidOutputLimit = clampFloat(value, 1.0f, 1000.0f);
}

void MotorConverter::setMaxPwm(float value) {
    maxPwm = clampFloat(value, 0.0f, 100.0f);

    if (motorStartPwm > maxPwm) {
        motorStartPwm = maxPwm;
    }
}

void MotorConverter::setMotorStartPwm(float value) {
    motorStartPwm = clampFloat(value, 0.0f, maxPwm);
}

void MotorConverter::setDeadband(float value) {
    deadband = clampFloat(value, 0.0f, 0.95f); 
}

void MotorConverter::setResponseCurve(float value) {
    responseCurve = clampFloat(value, 0.1f, 5.0f);
}

void MotorConverter::setDeadbandEnabled(bool enabled) {
    deadbandEnabled = enabled;
}

void MotorConverter::setResponseCurveEnabled(bool enabled) {
    responseCurveEnabled = enabled;
}

float MotorConverter::getPidOutputLimit() {
    return pidOutputLimit;
}

float MotorConverter::getMaxPwm() {
    return maxPwm;
}

float MotorConverter::getMotorStartPwm() {
    return motorStartPwm;
}

float MotorConverter::getDeadband() {
    return deadband;
}

float MotorConverter::getResponseCurve() {
    return responseCurve;
}

bool MotorConverter::isDeadbandEnabled() {
    return deadbandEnabled;
}

bool MotorConverter::isResponseCurveEnabled() {
    return responseCurveEnabled;
}

void MotorConverter::printSettings() {
    printf("\nMotor converter settings:\n");
    printf("pidOutputLimit = %.2f\n", pidOutputLimit);
    printf("maxPwm = %.2f\n", maxPwm);
    printf("motorStartPwm = %.2f\n", motorStartPwm);
    printf("deadband = %.4f\n", deadband);
    printf("deadbandEnabled = %d\n", deadbandEnabled);
    printf("responseCurve = %.4f\n", responseCurve);
    printf("responseCurveEnabled = %d\n\n", responseCurveEnabled);
}