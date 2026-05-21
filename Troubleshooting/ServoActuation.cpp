#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
    Standalone 4-servo actuation troubleshooting code.

    This file is for testing the servos without:
        - PID
        - motors
        - MPU6050
        - the main TentacleV2 control code

    Wiring:
        Servo 1 signal wire -> GP14
        Servo 2 signal wire -> GP15
        Servo 3 signal wire -> GP16
        Servo 4 signal wire -> GP17

    Servo power:
        Servo V+ wires  -> external 5V servo power supply
        Servo GND wires -> external power supply GND
        Pico GND        -> same external power supply GND

    Important:
        Do not power four servos from the Pico 3V3 pin.

    Failsafe:
        Each servo has its own estimated angle.
        Each servo stops by itself at -360 or +360 degrees.

    Super failsafe:
        Type FORCE to immediately stop all servos.
*/

const int SERVO_1_PIN = 14;
const int SERVO_2_PIN = 15;
const int SERVO_3_PIN = 16;
const int SERVO_4_PIN = 17;

const int SERVO_PINS[4] = {
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN,
    SERVO_4_PIN
};

const int PWM_WRAP = 20000 - 1;

const int SERVO_MIN_US = 1000;
const int SERVO_STOP_US = 1500;
const int SERVO_MAX_US = 2000;

const float MIN_SPEED = -100.0f;
const float MAX_SPEED = 100.0f;

const float MIN_REAL_ANGLE_DEGREES = -360.0f;
const float MAX_REAL_ANGLE_DEGREES = 360.0f;

/*
    Your angle command calibration:
        internal angle 22 -> real movement 90 degrees
*/
const float INTERNAL_DEGREES_FOR_90_REAL_DEGREES = 22.0f;

const float ANGLE_MOVE_SPEED = 25.0f;
const float MS_PER_DEGREE_AT_100_SPEED = 11.0f;

/*
    Your speed failsafe correction:
        the speed-mode estimate needed to count 3 times faster.
*/
const float SPEED_FAILSAFE_ANGLE_MULTIPLIER = 3.0f;

float servoSpeeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/*
    Separate estimated angle for each servo.
*/
float estimatedAngles[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool timedMoveRunning = false;
float timedMoveTargetAngle = 0.0f;
absolute_time_t timedMoveEndTime;

absolute_time_t lastAngleUpdateTime;

char inputBuffer[40];
int inputIndex = 0;

bool hasPendingInput = false;
absolute_time_t lastInputTime;

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

    printf("Setup servo PWM on GP%d\n", pin);
}

int speedToPulseUs(float speedPercent) {
    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

    int pulseUs = SERVO_STOP_US + (int)((speedPercent / 100.0f) * 500.0f);

    return clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
}

void setServoSpeedPin(int servoIndex, float speedPercent) {
    pwm_set_gpio_level(SERVO_PINS[servoIndex], speedToPulseUs(speedPercent));
}

void setOneServoSpeedByIndex(int servoIndex, float speedPercent) {
    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

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

void setAllServoSpeeds(float speedPercent) {
    for (int i = 0; i < 4; i++) {
        setOneServoSpeedByIndex(i, speedPercent);
    }
}

void stopServosQuietly() {
    setAllServoSpeeds(0.0f);
}

void printServoStatus() {
    printf("\n--- Servo status ---\n");

    for (int i = 0; i < 4; i++) {
        printf("Servo %d | GP%d | speed %.1f%% | pulse %d us | estimated angle %.1f\n",
               i + 1,
               SERVO_PINS[i],
               servoSpeeds[i],
               speedToPulseUs(servoSpeeds[i]),
               estimatedAngles[i]);
    }

    printf("Allowed range per servo: %.1f to %.1f degrees\n",
           MIN_REAL_ANGLE_DEGREES,
           MAX_REAL_ANGLE_DEGREES);
    printf("--------------------\n");
}

void forceStopServos() {
    /*
        Super failsafe.

        This immediately cancels any timed angle move and sends stop pulses to
        every servo. It does not care about estimated angles or normal command
        logic.
    */
    timedMoveRunning = false;

    for (int i = 0; i < 4; i++) {
        servoSpeeds[i] = 0.0f;
        setServoSpeedPin(i, 0.0f);
    }

    printf("\nFORCE STOP: all servos stopped immediately\n");
    printServoStatus();
}

void updateEstimatedAnglesAndFailsafes() {
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
            printServoStatus();
            printf("\nEnter command: ");
            fflush(stdout);
        } else if (estimatedAngles[i] <= MIN_REAL_ANGLE_DEGREES) {
            estimatedAngles[i] = MIN_REAL_ANGLE_DEGREES;
            servoSpeeds[i] = 0.0f;
            setServoSpeedPin(i, 0.0f);

            printf("\nSERVO %d FAILSAFE: minimum estimated angle reached. Servo stopped.\n", i + 1);
            printServoStatus();
            printf("\nEnter command: ");
            fflush(stdout);
        }
    }
}

void startTimedMove(float targetAngle, float angleDifference) {
    float moveSpeed = ANGLE_MOVE_SPEED;
    float absoluteDifference = angleDifference;

    if (absoluteDifference < 0.0f) {
        moveSpeed = -ANGLE_MOVE_SPEED;
        absoluteDifference = -absoluteDifference;
    }

    /*
        This is the working angle calibration.
        It is not multiplied by SPEED_FAILSAFE_ANGLE_MULTIPLIER.
    */
    float internalAngleDifference = absoluteDifference * (INTERNAL_DEGREES_FOR_90_REAL_DEGREES / 90.0f);
    float durationMs = internalAngleDifference * MS_PER_DEGREE_AT_100_SPEED * (100.0f / ANGLE_MOVE_SPEED);

    timedMoveRunning = true;
    timedMoveTargetAngle = targetAngle;
    timedMoveEndTime = make_timeout_time_ms((uint32_t)durationMs);

    setAllServoSpeeds(moveSpeed);

    printf("\nServos moving to %.1f estimated real degrees\n", targetAngle);
    printf("Servos rotating at %.1f%% for %.0f ms\n", moveSpeed, durationMs);
}

void updateTimedMove() {
    if (timedMoveRunning && absolute_time_diff_us(get_absolute_time(), timedMoveEndTime) <= 0) {
        stopServosQuietly();

        for (int i = 0; i < 4; i++) {
            estimatedAngles[i] = timedMoveTargetAngle;
        }

        timedMoveRunning = false;

        printf("\nServo timed angle move complete\n");
        printServoStatus();
        printf("\nEnter command: ");
        fflush(stdout);
    }
}

void setAllServosSpeed(float speedPercent) {
    updateEstimatedAnglesAndFailsafes();

    timedMoveRunning = false;
    setAllServoSpeeds(speedPercent);

    printf("\nAll servo speeds requested: %.1f%%\n", speedPercent);
    printServoStatus();
}

void setOneServoSpeed(int servoNumber, float speedPercent) {
    updateEstimatedAnglesAndFailsafes();

    timedMoveRunning = false;

    if (servoNumber < 1 || servoNumber > 4) {
        printf("\nUnknown servo number\n");
        return;
    }

    setOneServoSpeedByIndex(servoNumber - 1, speedPercent);

    printf("\nServo %d speed requested: %.1f%%\n", servoNumber, speedPercent);
    printServoStatus();
}

void stopServos() {
    updateEstimatedAnglesAndFailsafes();

    timedMoveRunning = false;
    stopServosQuietly();

    printf("\nServos stopped\n");
    printServoStatus();
}

void moveToAngle(float targetRealAngle) {
    updateEstimatedAnglesAndFailsafes();

    targetRealAngle = clampFloat(targetRealAngle, MIN_REAL_ANGLE_DEGREES, MAX_REAL_ANGLE_DEGREES);

    /*
        Use servo 1 as the reference for all-servo angle moves.
        Since all servos move together here, they are set to the same target
        when the timed move finishes.
    */
    float angleDifference = targetRealAngle - estimatedAngles[0];

    if (angleDifference > -0.5f && angleDifference < 0.5f) {
        printf("\nAlready close to %.1f real degrees\n", targetRealAngle);
        return;
    }

    startTimedMove(targetRealAngle, angleDifference);
}

void moveByAngle(float realAngleDifference) {
    updateEstimatedAnglesAndFailsafes();

    float targetRealAngle = estimatedAngles[0] + realAngleDifference;
    targetRealAngle = clampFloat(targetRealAngle, MIN_REAL_ANGLE_DEGREES, MAX_REAL_ANGLE_DEGREES);

    float correctedDifference = targetRealAngle - estimatedAngles[0];

    if (correctedDifference > -0.5f && correctedDifference < 0.5f) {
        printf("\nMove is too small or blocked by angle limit\n");
        return;
    }

    startTimedMove(targetRealAngle, correctedDifference);
}

void zeroAngleHere() {
    stopServosQuietly();

    for (int i = 0; i < 4; i++) {
        estimatedAngles[i] = 0.0f;
    }

    printf("\nAll current servo positions saved as 0 degrees\n");
    printServoStatus();
}

void printHelp() {
    printf("\nCommands:\n");
    printf("FORCE     -> SUPER FAILSAFE: immediately stop all servos\n");
    printf("angle 90  -> all servos go to estimated absolute +90 degrees\n");
    printf("move 90   -> all servos move +90 degrees from current position\n");
    printf("move -20  -> all servos move -20 degrees from current position\n");
    printf("zero      -> save all current servo positions as 0 degrees\n");
    printf("+20       -> rotate all servos at 20%% speed\n");
    printf("-20       -> rotate all servos the other direction\n");
    printf("s1 +25    -> set only servo 1 speed\n");
    printf("s2 -20    -> set only servo 2 speed\n");
    printf("s3 +25    -> set only servo 3 speed\n");
    printf("s4 -20    -> set only servo 4 speed\n");
    printf("stop      -> stop all servos\n");
    printf("status    -> print servo status\n");
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
    updateEstimatedAnglesAndFailsafes();

    inputBuffer[inputIndex] = '\0';

    if (inputIndex == 0) {
        clearInputBuffer();
        return;
    }

    printf("\nReceived command: %s\n", inputBuffer);

    if (strcmp(inputBuffer, "FORCE") == 0) {
        forceStopServos();
        clearInputBuffer();

        printf("\nEnter command: ");
        fflush(stdout);
        return;
    }

    char command[12];
    float value = 0.0f;

    int parts = sscanf(inputBuffer, "%11s %f", command, &value);

    if (parts >= 1) {
        if (strcmp(command, "angle") == 0 && parts == 2) {
            moveToAngle(value);
        } else if (strcmp(command, "move") == 0 && parts == 2) {
            moveByAngle(value);
        } else if (strcmp(command, "zero") == 0) {
            zeroAngleHere();
        } else if (strcmp(command, "stop") == 0 || strcmp(command, "0") == 0) {
            stopServos();
        } else if (strcmp(command, "status") == 0) {
            printServoStatus();
        } else if (strcmp(command, "help") == 0) {
            printHelp();
        } else if (strcmp(command, "s1") == 0 && parts == 2) {
            setOneServoSpeed(1, value);
        } else if (strcmp(command, "s2") == 0 && parts == 2) {
            setOneServoSpeed(2, value);
        } else if (strcmp(command, "s3") == 0 && parts == 2) {
            setOneServoSpeed(3, value);
        } else if (strcmp(command, "s4") == 0 && parts == 2) {
            setOneServoSpeed(4, value);
        } else if (command[0] == '+' || command[0] == '-' || (command[0] >= '0' && command[0] <= '9')) {
            setAllServosSpeed((float)atof(command));
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

    for (int i = 0; i < 4; i++) {
        setupServoPwm(SERVO_PINS[i]);
    }

    stopServosQuietly();
    clearInputBuffer();

    timedMoveEndTime = get_absolute_time();
    lastAngleUpdateTime = get_absolute_time();

    printf("\n4-servo actuation test ready\n");
    printHelp();
    printf("Enter command: ");
    fflush(stdout);

    while (true) {
        handleSerialInput();
        updateEstimatedAnglesAndFailsafes();
        updateTimedMove();
        sleep_ms(10);
    }
}