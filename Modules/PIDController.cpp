#include "PIDController.h"

/*
    Constructor.

    This runs when the PIDController object is created.

    The parameter names are also kp, ki, and kd, which are the same names as the
    class variables. Because of that, this-> is used.

    this->kp means:
        the kp variable that belongs to this PIDController object

    kp by itself means:
        the kp parameter passed into the constructor

    this-> use the variable that belongs to this object
*/
PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp; //Put the input kp value into this object's kp variable.
    this->ki = ki; //Put the input ki value into this object's ki variable.
    this->kd = kd; //Put the input kd value into this object's kd variable.

    /*
        Start with no old error and no accumulated integral.
    */
    previousError = 0.0f;
    integral = 0.0f;
}

/*
    Calculate the PID output.

    This function is called repeatedly in the main loop while PID is running.
*/
float PIDController::update(float setpoint, float measuredAngle, float measuredAngularVelocity, float dt, float &pValue, float &iValue, float &dValue) {
    /*
        error is how far away the measured value is from the target.

        Example:
            setpoint = 0
            measuredAngle = 5

            error = 0 - 5 = -5

        That means the system is 5 degrees away from the target in the negative
        correction direction.
    */
    float error = setpoint - measuredAngle;

    /*
        Integral term memory.

        This adds up error over time.

        If the tentacle is slightly off target for a long time, the integral
        grows and helps push it back. This is useful for correcting steady,
        persistent error.

        dt is included because the loop timing matters. Error for 0.01 seconds
        should count less than error for 1 full second.
    */
    // integral += error * dt;This is old and probabily does not work. Now trying to use the raw gyroscope data instead of integrating
    integral = measuredAngularVelocity;

    /*
        derivative estimates how quickly the error is changing.

        It starts at 0 in case dt is invalid.
    */
    float derivative = 0.0f;

    /*
        Only calculate derivative if dt is greater than zero.

        Dividing by zero would be a math error, so this check protects the code.
    */
    if (dt > 0.0f) {
        derivative = (error - previousError) / dt;
    }

    /*
        Calculate the three PID parts.

        P:
            Proportional. Reacts to the current error.

        I:
            Integral. Reacts to accumulated past error.

        D:
            Derivative. Reacts to how quickly the error is changing.
    */
    pValue = kp * error;
    iValue = ki * integral;
    dValue = kd * derivative;

    /*
        Save the current error so the next update can compare against it.
    */
    previousError = error;

    /*
        The final PID output is the sum of the three parts.

        Your main code later converts this number into motor direction and PWM.
    */
    return pValue + iValue + dValue;
}

/*
    Reset the controller memory.

    This does not change kp, ki, or kd.
    It only clears the stored previous error and integral buildup.
*/
void PIDController::reset() {
    previousError = 0.0f;
    integral = 0.0f;
}

/*
    Set a new proportional gain.

    Bigger Kp usually means stronger reaction to current error.
*/
void PIDController::setKp(float newKp) {
    kp = newKp;
}

/*
    Set a new integral gain.

    Bigger Ki usually means stronger correction for long-lasting error, but too
    much can make the system overshoot or become unstable.
*/
void PIDController::setKi(float newKi) {
    ki = newKi;
}

/*
    Set a new derivative gain.

    Bigger Kd usually means stronger damping, because it reacts to fast changes.
    Too much can make the motor response noisy or twitchy.
*/
void PIDController::setKd(float newKd) {
    kd = newKd;
}

/*
    Return the current Kp value.
*/
float PIDController::getKp() {
    return kp;
}

/*
    Return the current Ki value.
*/
float PIDController::getKi() {
    return ki;
}

/*
    Return the current Kd value.
*/
float PIDController::getKd() {
    return kd;
}