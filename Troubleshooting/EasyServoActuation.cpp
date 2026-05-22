#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

const float MIN_ANGLE_DEGREES = -360.0f;
const float MAX_ANGLE_DEGREES = 360.0f;

const float MS_PER_DEGREE_AT_100_SPEED = 11.0f;
const float SPEED_FAILSAFE_ANGLE_MULTIPLIER = 3.0f;

float servoSpeeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float estimatedAngles[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

void updateEstimatedAnglesAndFailsafes() {
    absolute_time_t now = get_absolute_time();
    float elapsedMs = absolute_time_diff_us(lastAngleUpdateTime, now) / 1000.0f;
    lastAngleUpdateTime = now;

    for (int i = 0; i < 4; i++) {
        if (servoSpeeds[i] > -0.01f && servoSpeeds[i] < 0.01f) {
            continue;
        }

        float degreesMoved = elapsedMs / MS_PER_DEGREE_AT_100_SPEED * (servoSpeeds[i] / 100.0f);
        degreesMoved = degreesMoved * SPEED_FAILSAFE_ANGLE_MULTIPLIER;

        estimatedAngles[i] += degreesMoved;

        if (estimatedAngles[i] >= MAX_ANGLE_DEGREES) {
            estimatedAngles[i] = MAX_ANGLE_DEGREES;
            servoSpeeds[i] = 0.0f;
            setServoSpeedPin(i, 0.0f);

            printf("\nServo %d stopped: maximum angle reached\n", i + 1);
        }

        if (estimatedAngles[i] <= MIN_ANGLE_DEGREES) {
            estimatedAngles[i] = MIN_ANGLE_DEGREES;
            servoSpeeds[i] = 0.0f;
            setServoSpeedPin(i, 0.0f);

            printf("\nServo %d stopped: minimum angle reached\n", i + 1);
        }
    }
}

void printStatus() {
    printf("\n--- Servo status ---\n");

    for (int i = 0; i < 4; i++) {
        printf("Servo %d | speed %.1f%% | estimated angle %.1f\n",
               i + 1,
               servoSpeeds[i],
               estimatedAngles[i]);
    }

    printf("--------------------\n");
}

void setOneServoSpeed(int servoNumber, float speedPercent) {
    updateEstimatedAnglesAndFailsafes();

    if (servoNumber < 1 || servoNumber > 4) {
        printf("\nUnknown servo number\n");
        return;
    }

    int servoIndex = servoNumber - 1;

    speedPercent = clampFloat(speedPercent, MIN_SPEED, MAX_SPEED);

    if (estimatedAngles[servoIndex] >= MAX_ANGLE_DEGREES && speedPercent > 0.0f) {
        speedPercent = 0.0f;
        printf("\nServo %d blocked: maximum angle reached\n", servoNumber);
    }

    if (estimatedAngles[servoIndex] <= MIN_ANGLE_DEGREES && speedPercent < 0.0f) {
        speedPercent = 0.0f;
        printf("\nServo %d blocked: minimum angle reached\n", servoNumber);
    }

    servoSpeeds[servoIndex] = speedPercent;
    setServoSpeedPin(servoIndex, speedPercent);

    printf("\nServo %d speed set to %.1f%%\n", servoNumber, speedPercent);
    printStatus();
}

void stopAllServos() {
    updateEstimatedAnglesAndFailsafes();

    for (int i = 0; i < 4; i++) {
        servoSpeeds[i] = 0.0f;
        setServoSpeedPin(i, 0.0f);
    }

    printf("\nAll servos stopped\n");
    printStatus();
}

void clearInputBuffer() {
    for (int i = 0; i < 40; i++) {
        inputBuffer[i] = '\0';
    }

    inputIndex = 0;
    hasPendingInput = false;
}

void printHelp() {
    printf("\nCommands:\n");
    printf("s1 20    -> set servo 1 speed\n");
    printf("s2 -20   -> set servo 2 speed\n");
    printf("s3 50    -> set servo 3 speed\n");
    printf("s4 -50   -> set servo 4 speed\n");
    printf("stop     -> stop all servos\n");
    printf("status   -> print servo status\n");
    printf("help     -> print this help text\n\n");
}

void processInput() {
    inputBuffer[inputIndex] = '\0';

    if (inputIndex == 0) {
        clearInputBuffer();
        return;
    }

    char command[12];
    float value = 0.0f;

    int parts = sscanf(inputBuffer, "%11s %f", command, &value);

    if (parts >= 1) {
        if (strcmp(command, "s1") == 0 && parts == 2) {
            setOneServoSpeed(1, value);
        } else if (strcmp(command, "s2") == 0 && parts == 2) {
            setOneServoSpeed(2, value);
        } else if (strcmp(command, "s3") == 0 && parts == 2) {
            setOneServoSpeed(3, value);
        } else if (strcmp(command, "s4") == 0 && parts == 2) {
            setOneServoSpeed(4, value);
        } else if (strcmp(command, "stop") == 0) {
            stopAllServos();
        } else if (strcmp(command, "status") == 0) {
            updateEstimatedAnglesAndFailsafes();
            printStatus();
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

    for (int i = 0; i < 4; i++) {
        setupServoPwm(SERVO_PINS[i]);
    }

    stopAllServos();
    clearInputBuffer();

    lastAngleUpdateTime = get_absolute_time();

    printf("\nEasy servo actuation ready\n");
    printHelp();
    printf("Enter command: ");
    fflush(stdout);

    while (true) {
        handleSerialInput();
        updateEstimatedAnglesAndFailsafes();
        sleep_ms(10);
    }
}