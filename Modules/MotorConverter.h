#pragma once

/*
    MotorCommand stores the result of converting PID output into motor PWM.

    pwmA:
        PWM for one motor direction.

    pwmB:
        PWM for the opposite motor direction.

    motorOutput:
        Signed motor output after scaling.
        Positive means direction A.
        Negative means direction B.

    normalizedOutput:
        PID output divided by pidOutputLimit, clamped from -1 to +1.

    scaledOutput:
        Absolute output after deadband and response curve.
        This is always 0 to 1.
*/
struct MotorCommand {
    float pwmA;
    float pwmB;
    float motorOutput;
    float normalizedOutput;
    float scaledOutput;
};

class MotorConverter {
public:
    MotorConverter(float pidOutputLimit, float maxPwm, float motorStartPwm);

    MotorCommand convert(float pidOutput);

    void setPidOutputLimit(float value);
    void setMaxPwm(float value);
    void setMotorStartPwm(float value);
    void setDeadband(float value);
    void setResponseCurve(float value);

    void setDeadbandEnabled(bool enabled);
    void setResponseCurveEnabled(bool enabled);

    float getPidOutputLimit();
    float getMaxPwm();
    float getMotorStartPwm();
    float getDeadband();
    float getResponseCurve();

    bool isDeadbandEnabled();
    bool isResponseCurveEnabled();

    void printSettings();

private:
    float pidOutputLimit;
    float maxPwm;
    float motorStartPwm;
    float deadband;
    float responseCurve;

    bool deadbandEnabled;
    bool responseCurveEnabled;

    float clampFloat(float value, float low, float high);
};