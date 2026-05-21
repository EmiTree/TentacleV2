#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int SERVO_1_PIN = 14;
const int SERVO_2_PIN = 15;

const int PWM_WRAP = 20000 - 1;

const int SERVO_MIN_US = 1000;
const int SERVO_STOP_US = 1500;
const int SERVO_MAX_US = 2000;

const float MIN_SPEED = -100.0f;
const float MAX_SPEED = 100.0f;

const float MIN_REAL_ANGLE_DEGREES = -360.0f;
const float MAX_REAL_ANGLE_DEGREES = 360.0f;

const float INTERNAL_DEGREES_FOR_90_REAL_DEGREES = 22.0f;

const float ANGLE_MOVE_SPEED = 25.0f;
const float MS_PER_DEGREE_AT_100_SPEED = 11.0f;

const float SPEED_FAILSAFE_ANGLE_MULTIPLIER = 3.0f;

float servo1Speed = 0.0f;
float servo2Speed = 0.0f;

float estimatedRealAngle = 0.0f;

char inputBuffer[40];
int inputIndex = 0;

bool hasPendingInput = false;
absolute_time_t lastInputTime;

absolute_time_t lastAngleUpdateTime;

float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

int clampInt(int value, int low, int high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void setupServoPwm(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);

    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, SERVO_STOP_US);

    printf("Setup continuous servo PWM on GP%d\n", pin);
}

int speedToPulseUs(float speedPercent) {
    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

    int pulseUs = SERVO_STOP_US + (int)((speedPercent / 100.0f) * 500.0f);

    return clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
}

void setServoSpeed(int pin, float speedPercent) {
    int pulseUs = speedToPulseUs(speedPercent);
    pwm_set_gpio_level(pin, pulseUs);
}

void setRawServoSpeeds(float speed1, float speed2) {
    servo1Speed = clampFloat(speed1, MIN_SPEED, MAX_SPEED);
    servo2Speed = clampFloat(speed2, MIN_SPEED, MAX_SPEED);

    setServoSpeed(SERVO_1_PIN, servo1Speed);
    setServoSpeed(SERVO_2_PIN, servo2Speed);

    lastAngleUpdateTime = get_absolute_time();
}

void stopServosQuietly() {
    setRawServoSpeeds(0.0f, 0.0f);
}

void printServoStatus() {
    printf("\n--- Continuous servo status ---\n");
    printf("Servo 1 | GP%d | speed %.1f%% | pulse %d us\n",
           SERVO_1_PIN,
           servo1Speed,
           speedToPulseUs(servo1Speed));
    printf("Servo 2 | GP%d | speed %.1f%% | pulse %d us\n",
           SERVO_2_PIN,
           servo2Speed,
           speedToPulseUs(servo2Speed));
    printf("Estimated real angle: %.1f degrees\n", estimatedRealAngle);
    printf("Allowed range: %.1f to %.1f degrees\n",
           MIN_REAL_ANGLE_DEGREES,
           MAX_REAL_ANGLE_DEGREES);
    printf("--------------------------------\n");
}

void updateEstimatedAngleAndFailsafe() {
    absolute_time_t now = get_absolute_time();
    float elapsedMs = absolute_time_diff_us(lastAngleUpdateTime, now) / 1000.0f;
    lastAngleUpdateTime = now;

    float averageSpeed = (servo1Speed + servo2Speed) / 2.0f;

    if (averageSpeed > -0.01f && averageSpeed < 0.01f) {
        return;
    }

    float internalDegreesMoved = elapsedMs / MS_PER_DEGREE_AT_100_SPEED * (averageSpeed / 100.0f);
    float realDegreesMoved = internalDegreesMoved * (90.0f / INTERNAL_DEGREES_FOR_90_REAL_DEGREES) * SPEED_FAILSAFE_ANGLE_MULTIPLIER;

    estimatedRealAngle += realDegreesMoved;

    if (estimatedRealAngle >= MAX_REAL_ANGLE_DEGREES) {
        estimatedRealAngle = MAX_REAL_ANGLE_DEGREES;
        stopServosQuietly();

        printf("\nFAILSAFE: reached maximum estimated angle %.1f degrees. Servos stopped.\n",
               MAX_REAL_ANGLE_DEGREES);
        printServoStatus();
        printf("\nEnter command: ");
        fflush(stdout);
    } else if (estimatedRealAngle <= MIN_REAL_ANGLE_DEGREES) {
        estimatedRealAngle = MIN_REAL_ANGLE_DEGREES;
        stopServosQuietly();

        printf("\nFAILSAFE: reached minimum estimated angle %.1f degrees. Servos stopped.\n",
               MIN_REAL_ANGLE_DEGREES);
        printServoStatus();
        printf("\nEnter command: ");
        fflush(stdout);
    }
}

void setServo1(float speedPercent) {
    updateEstimatedAngleAndFailsafe();

    setRawServoSpeeds(speedPercent, servo2Speed);

    printf("\nServo 1 speed set to %.1f%%\n", servo1Speed);
    printServoStatus();
}

void setServo2(float speedPercent) {
    updateEstimatedAngleAndFailsafe();

    setRawServoSpeeds(servo1Speed, speedPercent);

    printf("\nServo 2 speed set to %.1f%%\n", servo2Speed);
    printServoStatus();
}

void setBothServos(float speedPercent) {
    updateEstimatedAngleAndFailsafe();

    if (estimatedRealAngle >= MAX_REAL_ANGLE_DEGREES && speedPercent > 0.0f) {
        printf("\nBlocked: already at maximum estimated angle.\n");
        stopServosQuietly();
        printServoStatus();
        return;
    }

    if (estimatedRealAngle <= MIN_REAL_ANGLE_DEGREES && speedPercent < 0.0f) {
        printf("\nBlocked: already at minimum estimated angle.\n");
        stopServosQuietly();
        printServoStatus();
        return;
    }

    setRawServoSpeeds(speedPercent, speedPercent);

    printf("\nBoth servo speeds set to %.1f%%\n", servo1Speed);
    printServoStatus();
}

void stopServos() {
    updateEstimatedAngleAndFailsafe();

    stopServosQuietly();

    printf("\nBoth servos stopped\n");
    printServoStatus();
}

void moveToAngle(float targetRealAngle) {
    updateEstimatedAngleAndFailsafe();

    targetRealAngle = clampFloat(targetRealAngle, MIN_REAL_ANGLE_DEGREES, MAX_REAL_ANGLE_DEGREES);

    float realAngleDifference = targetRealAngle - estimatedRealAngle;

    if (realAngleDifference > -0.5f && realAngleDifference < 0.5f) {
        printf("\nAlready close to %.1f real degrees\n", targetRealAngle);
        return;
    }

    float moveSpeed = ANGLE_MOVE_SPEED;

    if (realAngleDifference < 0.0f) {
        moveSpeed = -ANGLE_MOVE_SPEED;
        realAngleDifference = -realAngleDifference;
    }

    float internalAngleDifference = realAngleDifference * (INTERNAL_DEGREES_FOR_90_REAL_DEGREES / 90.0f);
    float durationMs = internalAngleDifference * MS_PER_DEGREE_AT_100_SPEED * (100.0f / ANGLE_MOVE_SPEED);

    printf("\nMoving to estimated absolute angle %.1f real degrees\n", targetRealAngle);
    printf("Rotating at %.1f%% speed for %.0f ms\n", moveSpeed, durationMs);

    setRawServoSpeeds(moveSpeed, moveSpeed);
    sleep_ms((uint32_t)durationMs);

    stopServosQuietly();

    estimatedRealAngle = targetRealAngle;

    printServoStatus();
}

void moveByAngle(float realAngleDifference) {
    updateEstimatedAngleAndFailsafe();

    realAngleDifference = clampFloat(realAngleDifference,
                                     MIN_REAL_ANGLE_DEGREES,
                                     MAX_REAL_ANGLE_DEGREES);

    if (realAngleDifference > -0.5f && realAngleDifference < 0.5f) {
        printf("\nMove is too small\n");
        return;
    }

    float originalDifference = realAngleDifference;
    float moveSpeed = ANGLE_MOVE_SPEED;

    if (realAngleDifference < 0.0f) {
        moveSpeed = -ANGLE_MOVE_SPEED;
        realAngleDifference = -realAngleDifference;
    }

    float internalAngleDifference = realAngleDifference * (INTERNAL_DEGREES_FOR_90_REAL_DEGREES / 90.0f);
    float durationMs = internalAngleDifference * MS_PER_DEGREE_AT_100_SPEED * (100.0f / ANGLE_MOVE_SPEED);

    printf("\nMoving by %.1f real degrees\n", originalDifference);
    printf("Rotating at %.1f%% speed for %.0f ms\n", moveSpeed, durationMs);

    setRawServoSpeeds(moveSpeed, moveSpeed);
    sleep_ms((uint32_t)durationMs);

    stopServosQuietly();

    estimatedRealAngle += originalDifference;
    estimatedRealAngle = clampFloat(estimatedRealAngle,
                                    MIN_REAL_ANGLE_DEGREES,
                                    MAX_REAL_ANGLE_DEGREES);

    printServoStatus();
}

void zeroAngleHere() {
    stopServosQuietly();

    estimatedRealAngle = 0.0f;

    printf("\nCurrent position saved as 0 real degrees\n");
    printServoStatus();
}

void printHelp() {
    printf("\nCommands:\n");
    printf("angle 90  -> go to estimated absolute +90 degrees\n");
    printf("move 90   -> move +90 degrees from current position\n");
    printf("move -20  -> move -20 degrees from current position\n");
    printf("zero      -> save current position as 0 degrees\n");
    printf("0         -> stop both servos\n");
    printf("+20       -> rotate at 20%% speed until stopped or failsafe reached\n");
    printf("-20       -> rotate other direction at 20%% speed until stopped or failsafe reached\n");
    printf("s1 +25    -> set only servo 1 speed\n");
    printf("s2 -20    -> set only servo 2 speed\n");
    printf("both 10   -> set both servo speeds\n");
    printf("stop      -> stop both servos\n");
    printf("status    -> print current values\n");
    printf("help      -> print this help text\n\n");
}

void clearInputBuffer() {
    for (int i = 0; i < 40; i++) {
        inputBuffer[i] = '\0';
    }

    inputIndex = 0;
    hasPendingInput = false;
}

void processInput() {
    updateEstimatedAngleAndFailsafe();

    inputBuffer[inputIndex] = '\0';

    if (inputIndex == 0) {
        clearInputBuffer();
        return;
    }

    printf("\nReceived command: %s\n", inputBuffer);

    char command[12];
    float value = 0.0f;

    int parts = sscanf(inputBuffer, "%11s %f", command, &value);

    if (parts >= 1) {
        if (strcmp(command, "move") == 0 && parts == 2) {
            moveByAngle(value);
        } else if (strcmp(command, "angle") == 0 && parts == 2) {
            moveToAngle(value);
        } else if (strcmp(command, "zero") == 0) {
            zeroAngleHere();
        } else if (command[0] == '+' || command[0] == '-' || (command[0] >= '0' && command[0] <= '9')) {
            setBothServos((float)atof(command));
        } else if (strcmp(command, "s1") == 0 && parts == 2) {
            setServo1(value);
        } else if (strcmp(command, "s2") == 0 && parts == 2) {
            setServo2(value);
        } else if (strcmp(command, "both") == 0 && parts == 2) {
            setBothServos(value);
        } else if (strcmp(command, "stop") == 0) {
            stopServos();
        } else if (strcmp(command, "status") == 0) {
            printServoStatus();
        } else if (strcmp(command, "help") == 0) {
            printHelp();
        } else {
            printf("\nUnknown command: %s\n", inputBuffer);
            printHelp();
        }
    }

    clearInputBuffer();

    printf("\nEnter command: ");
    fflush(stdout);
}

void handleSerialInput() {
    int ch = getchar_timeout_us(0);

    if (ch == PICO_ERROR_TIMEOUT) {
        if (hasPendingInput) {
            absolute_time_t now = get_absolute_time();
            int64_t timeSinceInput = absolute_time_diff_us(lastInputTime, now);

            if (timeSinceInput > 500000) {
                processInput();
            }
        }

        return;
    }

    if (ch == '\r' || ch == '\n') {
        processInput();
        return;
    }

    if (ch == 8 || ch == 127) {
        if (inputIndex > 0) {
            inputIndex--;
            inputBuffer[inputIndex] = '\0';
        }

        hasPendingInput = true;
        lastInputTime = get_absolute_time();
        return;
    }

    if (inputIndex < 39) {
        inputBuffer[inputIndex] = (char)ch;
        inputIndex++;
        inputBuffer[inputIndex] = '\0';

        hasPendingInput = true;
        lastInputTime = get_absolute_time();

        printf("%c", ch);
        fflush(stdout);
    } else {
        printf("\nInput too long, clearing input\n");
        clearInputBuffer();
        printf("Enter command: ");
        fflush(stdout);
    }
}

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    setupServoPwm(SERVO_1_PIN);
    setupServoPwm(SERVO_2_PIN);

    stopServosQuietly();
    clearInputBuffer();

    lastAngleUpdateTime = get_absolute_time();

    printf("\nContinuous servo command test ready\n");
    printHelp();
    printf("Enter command: ");
    fflush(stdout);

    while (true) {
        handleSerialInput();
        updateEstimatedAngleAndFailsafe();
        sleep_ms(10);
    }
}