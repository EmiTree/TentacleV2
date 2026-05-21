#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

/*
    Motor + MPU debug code.

    This tests the motors and MPU together, but without:
        - PID
        - servos
        - the full TentacleV2 main code

    The goal is to figure out whether the motors and MPU can work at the
    same time, or whether motor power/noise/wiring is disturbing the MPU.
*/

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

const int PWM_TOP = 1000;
const float MAX_SPEED = 100.0f;

#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

#define ACCEL_SCALE_FACTOR 8192.0f
#define GYRO_SCALE_FACTOR 131.0f

#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

const float PI_VALUE = 3.14159265359f;

/*
    I2C timeout.

    Normal i2c_write_blocking() can freeze forever if the I2C bus is stuck.
    The timeout versions return an error instead.

    100000 microseconds = 100 milliseconds.
*/
const uint I2C_TIMEOUT_US = 100000;

float speed = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;

bool mpuOk = false;

char inputBuffer[40];
int inputIndex = 0;

absolute_time_t lastInputTime;
absolute_time_t lastPrintTime;
absolute_time_t lastAngleTime;

bool hasPendingInput = false;

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

void setupPwmPin(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_TOP);

    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0);

    printf("Setup motor PWM pin GP%d\n", pin);
}

void applyMotorSpeed(float speedPercent) {
    speed = clampFloat(speedPercent, -MAX_SPEED, MAX_SPEED);

    int pwmValue = (int)((speed < 0 ? -speed : speed) / 100.0f * PWM_TOP);
    pwmValue = clampInt(pwmValue, 0, PWM_TOP);

    if (speed > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, pwmValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwmValue);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    } else if (speed < 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, pwmValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, pwmValue);
    } else {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    }
}

void stopMotors() {
    applyMotorSpeed(0.0f);
}

/*
    Write one value to one MPU register.

    Returns true if the write worked.
    Returns false if the write failed or timed out.
*/
bool mpuWriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};

    int result = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        data,
        2,
        false,
        I2C_TIMEOUT_US
    );

    return result == 2;
}

/*
    Read one value from one MPU register.

    This is used for WHO_AM_I.
*/
bool mpuReadRegister(uint8_t reg, uint8_t *value) {
    int writeResult = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        value,
        1,
        false,
        I2C_TIMEOUT_US
    );

    return readResult == 1;
}

/*
    Read raw accelerometer and gyroscope data from the MPU.

    Returns false if I2C fails.
*/
bool mpuReadRaw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    int writeResult = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        buffer,
        14,
        false,
        I2C_TIMEOUT_US
    );

    if (readResult != 14) {
        return false;
    }

    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);

    return true;
}

/*
    Set up the MPU6050.

    This now checks every important I2C step, so if something fails you should
    see where it failed instead of the program freezing forever.
*/
bool setupMpu() {
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(100);

    if (!mpuWriteRegister(REG_PWR_MGMT_1, 0x80)) {
        printf("MPU reset write failed\n");
        return false;
    }
    sleep_ms(200);

    if (!mpuWriteRegister(REG_PWR_MGMT_1, 0x00)) {
        printf("MPU wake write failed\n");
        return false;
    }
    sleep_ms(200);

    uint8_t whoAmI = 0;
    bool readOk = mpuReadRegister(WHO_AM_I_REG, &whoAmI);

    printf("MPU WHO_AM_I readOk=%d value=0x%02X\n", readOk, whoAmI);

    if (!readOk || whoAmI != 0x68) {
        return false;
    }

    if (!mpuWriteRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE)) {
        printf("MPU accel config write failed\n");
        return false;
    }

    if (!mpuWriteRegister(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE)) {
        printf("MPU gyro config write failed\n");
        return false;
    }

    if (!mpuWriteRegister(REG_SMPLRT_DIV, SAMPLE_RATE_DIV)) {
        printf("MPU sample rate write failed\n");
        return false;
    }

    return true;
}

void updateMpuAngle() {
    if (!mpuOk) {
        return;
    }

    int16_t accel[3];
    int16_t gyro[3];

    bool readOk = mpuReadRaw(accel, gyro);

    if (!readOk) {
        mpuOk = false;
        stopMotors();

        printf("\nMPU READ FAILED. Motors stopped for safety.\n");
        printf("This points to wiring, power noise, or I2C disturbance.\n");
        return;
    }

    float ax = accel[0] / ACCEL_SCALE_FACTOR;
    float ay = accel[1] / ACCEL_SCALE_FACTOR;
    float az = accel[2] / ACCEL_SCALE_FACTOR;

    float gx = gyro[0] / GYRO_SCALE_FACTOR;
    float gy = gyro[1] / GYRO_SCALE_FACTOR;

    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(lastAngleTime, now) / 1000000.0f;
    lastAngleTime = now;

    float rollAcc = atan2f(ay, az) * 180.0f / PI_VALUE;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

    roll += gx * dt;
    pitch += gy * dt;

    roll = 0.98f * roll + 0.02f * rollAcc;
    pitch = 0.98f * pitch + 0.02f * pitchAcc;
}

void printStatus() {
    printf("\n--- Motor + MPU debug ---\n");
    printf("Motor speed: %.1f%%\n", speed);
    printf("MPU ok: %d\n", mpuOk);
    printf("roll: %.2f | pitch: %.2f\n", roll, pitch);
    printf("-------------------------\n");
}

void printHelp() {
    printf("\nCommands:\n");
    printf("speed 30    -> run motors forward\n");
    printf("speed -30   -> run motors backward\n");
    printf("stop        -> stop motors\n");
    printf("status      -> print motor and MPU status\n");
    printf("help        -> print this help text\n\n");
}

void clearInputBuffer() {
    for (int i = 0; i < 40; i++) {
        inputBuffer[i] = '\0';
    }

    inputIndex = 0;
    hasPendingInput = false;
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
        if (strcmp(command, "speed") == 0 && parts == 2) {
            applyMotorSpeed(value);
            printf("\nMotor speed set to %.1f%%\n", speed);
            printStatus();
        } else if (strcmp(command, "stop") == 0) {
            stopMotors();
            printf("\nMotors stopped\n");
            printStatus();
        } else if (strcmp(command, "status") == 0) {
            printStatus();
        } else if (strcmp(command, "help") == 0) {
            printHelp();
        } else {
            printf("\nUnknown command: %s\n", inputBuffer);
            printHelp();
        }
    }

    clearInputBuffer();

    printf("Enter command: ");
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

    printf("Motor + MPU debug starting...\n");

    setupPwmPin(MOTOR_PIN_P_1);
    setupPwmPin(MOTOR_PIN_P_2);
    setupPwmPin(MOTOR_PIN_Q_1);
    setupPwmPin(MOTOR_PIN_Q_2);

    stopMotors();

    mpuOk = setupMpu();

    if (!mpuOk) {
        printf("MPU setup failed. Motors will still be testable, but check MPU wiring/power.\n");
    } else {
        printf("MPU setup ok.\n");
    }

    clearInputBuffer();

    lastPrintTime = get_absolute_time();
    lastAngleTime = get_absolute_time();

    printHelp();
    printf("Enter command: ");
    fflush(stdout);

    while (true) {
        handleSerialInput();
        updateMpuAngle();

        absolute_time_t now = get_absolute_time();
        int64_t timeSincePrint = absolute_time_diff_us(lastPrintTime, now);

        if (timeSincePrint > 1000000) {
            lastPrintTime = now;

            printf("speed=%.1f | mpu=%d | roll=%.2f | pitch=%.2f\n",
                   speed,
                   mpuOk,
                   roll,
                   pitch);
        }

        sleep_ms(10);
    }
}