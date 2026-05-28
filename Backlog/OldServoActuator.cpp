#include "ServoActuator.h"
#include "hardware/pwm.h"

#include <stdio.h>

const int PWM_WRAP = 20000 - 1;

const int SERVO_MIN_US = 1000;
const int SERVO_STOP_US = 1500;
const int SERVO_MAX_US = 2000;

const float MIN_SPEED = -100.0f;
const float MAX_SPEED = 100.0f;

float MIN_REAL_ANGLE_DEGREES = -1000000000.0f;
float MAX_REAL_ANGLE_DEGREES = 1000000000.0f;

const float INTERNAL_DEGREES_FOR_90_REAL_DEGREES = 22.0f;
const float ANGLE_MOVE_SPEED = 25.0f;
const float MS_PER_DEGREE_AT_100_SPEED = 11.0f;

/*
    Your speed failsafe correction.

    This means speed-mode angle estimation counts 3 times faster, matching your
    real test where the servo moved 3 times farther than the first estimate.
*/
const float SPEED_FAILSAFE_ANGLE_MULTIPLIER = 3.0f;

ServoActuator::ServoActuator(int servo1Pin, int servo2Pin, int servo3Pin, int servo4Pin) {
    servoPins[0] = servo1Pin;
    servoPins[1] = servo2Pin;
    servoPins[2] = servo3Pin;
    servoPins[3] = servo4Pin;

    for (int i = 0; i < 4; i++) {
        servoSpeeds[i] = 0.0f;
        estimatedAngles[i] = 0.0f;
    }

    timedMoveRunning = false;
    timedMoveTargetAngle = 0.0f;
    timedMoveEndTime = get_absolute_time();
    lastAngleUpdateTime = get_absolute_time();
}

void ServoActuator::begin() {
    for (int i = 0; i < 4; i++) {
        setupServoPwm(servoPins[i]);
    }

    stop();
    lastAngleUpdateTime = get_absolute_time();

    printf("Servo actuator ready on GP%d, GP%d, GP%d, GP%d\n",
           servoPins[0], servoPins[1], servoPins[2], servoPins[3]);
}

void ServoActuator::update() {
    updateEstimatedAnglesAndFailsafes();

    if (timedMoveRunning && absolute_time_diff_us(get_absolute_time(), timedMoveEndTime) <= 0) {
        setAllServoSpeeds(0.0f);

        for (int i = 0; i < 4; i++) {
            estimatedAngles[i] = timedMoveTargetAngle;
        }

        timedMoveRunning = false;

        printf("\nServo timed angle move complete\n");
        printStatus();
    }
}

float ServoActuator::clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

int ServoActuator::clampInt(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void ServoActuator::setupServoPwm(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);

    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, SERVO_STOP_US);
}

int ServoActuator::speedToPulseUs(float speedPercent) {
    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

    int pulseUs = SERVO_STOP_US + (int)((speedPercent / 100.0f) * 500.0f);

    return clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
}

void ServoActuator::setServoSpeedPin(int servoIndex, float speedPercent) {
    pwm_set_gpio_level(servoPins[servoIndex], speedToPulseUs(speedPercent));
}

void ServoActuator::setOneServoSpeedByIndex(int servoIndex, float speedPercent) {
    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

    /*
        Individual failsafe.

        This blocks only the servo that is already at its own limit.
    */
    if (estimatedAngles[servoIndex] >= MAX_REAL_ANGLE_DEGREES && speedPercent > 0.0f) {
        speedPercent = 0.0f;
        printf("\nServo %d blocked: maximum estimated angle reached\n", servoIndex + 1);
    }

    if (estimatedAngles[servoIndex] <= MIN_REAL_ANGLE_DEGREES && speedPercent < 0.0f) {
        speedPercent = 0.0f;
        printf("\nServo %d blocked: minimum estimated angle reached\n", servoIndex + 1);
    }

    servoSpeeds[servoIndex] = speedPercent;
    setServoSpeedPin(servoIndex, speedPercent);

    lastAngleUpdateTime = get_absolute_time();
}

void ServoActuator::setAllServoSpeeds(float speedPercent) {
    for (int i = 0; i < 4; i++) {
        setOneServoSpeedByIndex(i, speedPercent);
    }
}

void ServoActuator::setSpeed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();

    timedMoveRunning = false;
    setAllServoSpeeds(speedPercent);

    printf("\nAll servo speeds requested: %.1f%%\n", speedPercent);
    printStatus();
}

void ServoActuator::setServo1Speed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();
    timedMoveRunning = false;
    setOneServoSpeedByIndex(0, speedPercent);
    printStatus();
}

void ServoActuator::setServo2Speed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();
    timedMoveRunning = false;
    setOneServoSpeedByIndex(1, speedPercent);
    printStatus();
}

void ServoActuator::setServo3Speed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();
    timedMoveRunning = false;
    setOneServoSpeedByIndex(2, speedPercent);
    printStatus();
}

void ServoActuator::setServo4Speed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();
    timedMoveRunning = false;
    setOneServoSpeedByIndex(3, speedPercent);
    printStatus();
}

void ServoActuator::stop() {
    timedMoveRunning = false;
    setAllServoSpeeds(0.0f);

    printf("\nServos stopped\n");
}

void ServoActuator::moveToAngle(float targetRealAngle) {
    updateEstimatedAnglesAndFailsafes();

    targetRealAngle = clampFloat(targetRealAngle, MIN_REAL_ANGLE_DEGREES, MAX_REAL_ANGLE_DEGREES);

    /*
        For all-servo angle moves, use servo 1 as the reference estimate.
        Since all four move together in this command, they are then set to the
        same target when the timed move completes.
    */
    float angleDifference = targetRealAngle - estimatedAngles[0];

    if (angleDifference > -0.5f && angleDifference < 0.5f) {
        printf("\nServos already close to %.1f degrees\n", targetRealAngle);
        return;
    }

    startTimedMove(targetRealAngle, angleDifference);
}

void ServoActuator::moveByAngle(float realAngleDifference) {
    updateEstimatedAnglesAndFailsafes();

    float targetRealAngle = estimatedAngles[0] + realAngleDifference;
    targetRealAngle = clampFloat(targetRealAngle, MIN_REAL_ANGLE_DEGREES, MAX_REAL_ANGLE_DEGREES);

    float correctedDifference = targetRealAngle - estimatedAngles[0];

    if (correctedDifference > -0.5f && correctedDifference < 0.5f) {
        printf("\nServo move is too small or blocked by angle limit\n");
        return;
    }

    startTimedMove(targetRealAngle, correctedDifference);
}

void ServoActuator::startTimedMove(float targetAngle, float angleDifference) {
    float moveSpeed = ANGLE_MOVE_SPEED;
    float absoluteDifference = angleDifference;

    if (absoluteDifference < 0.0f) {
        moveSpeed = -ANGLE_MOVE_SPEED;
        absoluteDifference = -absoluteDifference;
    }

    float internalAngleDifference = absoluteDifference * (INTERNAL_DEGREES_FOR_90_REAL_DEGREES / 90.0f);
    float durationMs = internalAngleDifference * MS_PER_DEGREE_AT_100_SPEED * (100.0f / ANGLE_MOVE_SPEED);

    timedMoveRunning = true;
    timedMoveTargetAngle = targetAngle;
    timedMoveEndTime = make_timeout_time_ms((uint32_t)durationMs);

    setAllServoSpeeds(moveSpeed);

    printf("\nServos moving to %.1f estimated real degrees\n", targetAngle);
    printf("Servos rotating at %.1f%% for %.0f ms\n", moveSpeed, durationMs);
}

void ServoActuator::zeroAngleHere() {
    stop();

    for (int i = 0; i < 4; i++) {
        estimatedAngles[i] = 0.0f;
    }

    printf("\nAll current servo positions saved as 0 degrees\n");
    printStatus();
}

void ServoActuator::updateEstimatedAnglesAndFailsafes() {
    absolute_time_t now = get_absolute_time();
    float elapsedMs = absolute_time_diff_us(lastAngleUpdateTime, now) / 1000.0f;
    lastAngleUpdateTime = now;

    for (int i = 0; i < 4; i++) {
        if (servoSpeeds[i] > -0.01f && servoSpeeds[i] < 0.01f) {
            continue;
        }

        float internalDegreesMoved = elapsedMs / MS_PER_DEGREE_AT_100_SPEED * (servoSpeeds[i] / 100.0f);
        float realDegreesMoved = internalDegreesMoved * (90.0f / INTERNAL_DEGREES_FOR_90_REAL_DEGREES) * SPEED_FAILSAFE_ANGLE_MULTIPLIER;

        estimatedAngles[i] += realDegreesMoved;

        if (estimatedAngles[i] >= MAX_REAL_ANGLE_DEGREES) {
            estimatedAngles[i] = MAX_REAL_ANGLE_DEGREES;
            servoSpeeds[i] = 0.0f;
            setServoSpeedPin(i, 0.0f);

            printf("\nSERVO %d FAILSAFE: maximum estimated angle reached. Servo stopped.\n", i + 1);
        } else if (estimatedAngles[i] <= MIN_REAL_ANGLE_DEGREES) {
            estimatedAngles[i] = MIN_REAL_ANGLE_DEGREES;
            servoSpeeds[i] = 0.0f;
            setServoSpeedPin(i, 0.0f);

            printf("\nSERVO %d FAILSAFE: minimum estimated angle reached. Servo stopped.\n", i + 1);
        }
    }
}

void ServoActuator::printStatus() {
    printf("\n--- Servo status ---\n");

    for (int i = 0; i < 4; i++) {
        printf("Servo %d | GP%d | speed %.1f%% | pulse %d us | estimated angle %.1f\n",
               i + 1,
               servoPins[i],
               servoSpeeds[i],
               speedToPulseUs(servoSpeeds[i]),
               estimatedAngles[i]);
    }

    printf("Allowed range per servo: %.1f to %.1f degrees\n",
           MIN_REAL_ANGLE_DEGREES,
           MAX_REAL_ANGLE_DEGREES);
    printf("--------------------\n");
}

void ServoActuator::setMinAngle(float minAngle) {
    MIN_REAL_ANGLE_DEGREES = minAngle;

    if (MIN_REAL_ANGLE_DEGREES > MAX_REAL_ANGLE_DEGREES) {
        MAX_REAL_ANGLE_DEGREES = MIN_REAL_ANGLE_DEGREES;
    }

    printf("\nServo minimum angle set to %.1f degrees\n", MIN_REAL_ANGLE_DEGREES);
    printStatus();
}

void ServoActuator::setMaxAngle(float maxAngle) {
    MAX_REAL_ANGLE_DEGREES = maxAngle;

    if (MAX_REAL_ANGLE_DEGREES < MIN_REAL_ANGLE_DEGREES) {
        MIN_REAL_ANGLE_DEGREES = MAX_REAL_ANGLE_DEGREES;
    }

    printf("\nServo maximum angle set to %.1f degrees\n", MAX_REAL_ANGLE_DEGREES);
    printStatus();
}

void ServoActuator::printHelp() {
    printf("\nServo commands:\n");
    printf("servo angle 90  -> all servos go to estimated absolute +90 degrees\n");
    printf("servo move 90   -> all servos move +90 degrees from current position\n");
    printf("servo zero      -> save all current servo positions as 0 degrees\n");
    printf("servo +20       -> rotate all servos at 20%% speed\n");
    printf("servo -20       -> rotate all servos the other direction\n");
    printf("servo s1 +25    -> set only servo 1 speed\n");
    printf("servo s2 -20    -> set only servo 2 speed\n");
    printf("servo s3 +25    -> set only servo 3 speed\n");
    printf("servo s4 -20    -> set only servo 4 speed\n");
    printf("servo stop      -> stop all servos\n");
    printf("servo status    -> print servo status\n");
}