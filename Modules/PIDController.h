#pragma once

/*
    PIDController is a C++ class.

    A class is like a blueprint for an object. In your main code, you create one
    PIDController object like this:

        PIDController pid(5.0f, 0.01f, 0.5f);

    That object remembers:
    - the PID tuning values: kp, ki, kd
    - the previous error
    - the accumulated integral error
*/
class PIDController {
public:
    /*
        Constructor.

        This function runs when a PIDController object is first created.

        Example:
            PIDController pid(5.0f, 0.01f, 0.5f);

        That means:
            kp starts as 5.0
            ki starts as 0.01
            kd starts as 0.5
    */
    PIDController(float kp, float ki, float kd);

    /*
        Calculate one PID update.

        Inputs:
            setpoint:
                The target value you want. For your tentacle, this is the target angle.

            measuredValue:
                The current measured value. For your tentacle, this is the measured angle.

            dt:
                Time since the previous update, in seconds.

            pValue, iValue, dValue:
                These use & references, meaning this function can fill in the original
                variables from the main code. That lets your main code print the separate
                P, I, and D parts for debugging.

        Returns:
            The final PID output:
                P + I + D
    */
    float update(float setpoint, float measuredAngle, float measuredAngularVelocity,  float dt, float &pValue, float &iValue, float &dValue);

    /*
        Reset the PID memory.

        This clears:
        - previousError
        - integral

        It is useful when stopping, starting, or changing PID settings.
    */
    void reset();

    /*
        Change the PID tuning values while the program is running.

        Your serial commands use these functions when you type:
            kp 5
            ki 0.01
            kd 0.5
    */
    void setKp(float newKp);
    void setKi(float newKi);
    void setKd(float newKd);

    /*
        Read the current PID tuning values.

        Your printSettings() function uses these so it can print the current
        Kp, Ki, and Kd values.
    */
    float getKp();
    float getKi();
    float getKd();

private:
    /*
        private means only this class can directly use these variables.

        Other code cannot directly do:
            pid.kp = 10;

        Instead, other code should use:
            pid.setKp(10);
    */

    /*
        PID tuning constants.

        kp controls proportional response.
        ki controls integral response.
        kd controls derivative response.
    */
    float kp;
    float ki;
    float kd;

    /*
        previousError stores the error from the previous update.

        This is needed to estimate the derivative, which means:
            how quickly is the error changing?
    */
    float previousError;

    /*
        integral stores accumulated error over time.

        This helps correct small errors that stay around for a long time.
    */
    float integral;
};