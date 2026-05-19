#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

//Variables
const float KP = 5.0;
const float KI = 0.01;
const float KD = 0.5;
const float SETPOINT = 0.0;

float previousError = 0.0;
float integral = 0.0;

bool pidRunning = false;

char commandBuffer[20];
int commandIndex = 0;
bool hasPendingCommand = false;
absolute_time_t lastCommandTime;
absolute_time_t lastStatusPrint;

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

const float MAX_PID_OUTPUT = 140.0;
const float MAX_PWM = 80.0;
const float MOTOR_START_PWM = 20.0;

#define LED_GREEN 13
#define LED_RED 12

#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68

#define SERVO_PIN 14
#define MAXIMUM_LEVEL 1000

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

#define ACCEL_SCALE_FACTOR_4G 8192.0
#define GYRO_SCALE_FACTOR_250DPS 131.0

#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS

#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

const float PI_VALUE = 3.14159265359f;

float constrain(float value, float minVal, float maxVal);
void setupMotorPWM(int pin);
float updatePid(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue);
void convertPidToMotor(float pidOutput, float &pwmA, float &pwmB, float &motorOutput);
float addMotorStartPower(float pwm);
void driveMotors(float pwmA, float pwmB);
void stopMotors();
void resetPid();
void mpu6050_reset();
void mpu6050_configure();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
void clearCommandBuffer();
void processCommand();
void handleSerialCommands();

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Starting...\n");

    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);

    setupMotorPWM(MOTOR_PIN_P_1);
    setupMotorPWM(MOTOR_PIN_P_2);
    setupMotorPWM(MOTOR_PIN_Q_1);
    setupMotorPWM(MOTOR_PIN_Q_2);
    stopMotors();

    printf("Successfully setup motors\n");

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    printf("Initialized I2C port\n");

    mpu6050_reset();
    mpu6050_configure();

    printf("Configured MPU6050\n");

    uint8_t who_am_i = 0;
    uint8_t who_reg = WHO_AM_I_REG;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &who_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);

    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        while (true) {
            printf("MPU6050 not found!\n");
            stopMotors();
            sleep_ms(1000);
        }
    }

    int16_t accel[3], gyro[3], temp;

    float roll = 0.0f;
    float pitch = 0.0f;

    absolute_time_t last_time = get_absolute_time();

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);
    pwm_init(slice_num, &config, true);

    clearCommandBuffer();

    printf("PID is stopped.\n");
    printf("Type 'start' to run PID.\n");
    printf("Type 'stop' to stop motors.\n");
    printf("Command: ");
    fflush(stdout);

    lastStatusPrint = get_absolute_time();

    while (true) {
        handleSerialCommands();

        mpu6050_read_raw(accel, gyro, &temp);

        float ax = accel[0] / ACCEL_SCALE_FACTOR;
        float ay = accel[1] / ACCEL_SCALE_FACTOR;
        float az = accel[2] / ACCEL_SCALE_FACTOR;

        float gx = gyro[0] / GYRO_SCALE_FACTOR;
        float gy = gyro[1] / GYRO_SCALE_FACTOR;

        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1000000.0f;
        last_time = current_time;

        float roll_acc = atan2f(ay, az) * 180.0f / PI_VALUE;
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

        roll += gx * dt;
        pitch += gy * dt;

        roll = 0.98f * roll + 0.02f * roll_acc;
        pitch = 0.98f * pitch + 0.02f * pitch_acc;

        float roll_out = roll + 2.3f;
        float pitch_out = pitch + 4.4f;

        float pValue = 0.0f;
        float iValue = 0.0f;
        float dValue = 0.0f;

        float pidOutput = 0.0f;
        float pwmA = 0.0f;
        float pwmB = 0.0f;
        float motorOutput = 0.0f;

        if (pidRunning) {
            pidOutput = updatePid(SETPOINT, roll_out, dt, pValue, iValue, dValue);

            convertPidToMotor(pidOutput, pwmA, pwmB, motorOutput);

            driveMotors(pwmA, pwmB);
        } else {
            stopMotors();
        }

        absolute_time_t now = get_absolute_time();
        int64_t time_since_status = absolute_time_diff_us(lastStatusPrint, now);

        if (time_since_status > 500000) {
            lastStatusPrint = now;

            printf("\nstate = %s | roll = %.2f | pitch = %.2f | pid = %.2f | pwmA = %.2f | pwmB = %.2f\n",
                   pidRunning ? "running" : "stopped",
                   roll_out,
                   pitch_out,
                   pidOutput,
                   pwmA,
                   pwmB);

            printf("Command: ");
            fflush(stdout);
        }

        sleep_ms(10);
    }

    return 0;
}

float constrain(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void setupMotorPWM(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

float updatePid(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue) {
    float error = setpoint - measuredValue;

    integral += error * dt;

    float derivative = 0.0f;

    if (dt > 0.0f) {
        derivative = (error - previousError) / dt;
    }

    pValue = KP * error;
    iValue = KI * integral;
    dValue = KD * derivative;

    previousError = error;

    return pValue + iValue + dValue;
}

void convertPidToMotor(float pidOutput, float &pwmA, float &pwmB, float &motorOutput) {
    motorOutput = pidOutput / MAX_PID_OUTPUT * MAX_PWM;
    motorOutput = constrain(motorOutput, -MAX_PWM, MAX_PWM);

    if (motorOutput > 0.0f) {
        pwmA = motorOutput;
        pwmB = 0.0f;
    } else if (motorOutput < 0.0f) {
        pwmA = 0.0f;
        pwmB = -motorOutput;
    } else {
        pwmA = 0.0f;
        pwmB = 0.0f;
    }
}

float addMotorStartPower(float pwm) {
    if (pwm <= 0.0f) {
        return 0.0f;
    }

    float adjustedPwm = MOTOR_START_PWM + pwm;

    return constrain(adjustedPwm, 0.0f, MAX_PWM);
}

void driveMotors(float pwmA, float pwmB) {
    float adjustedPwmA = addMotorStartPower(pwmA);
    float adjustedPwmB = addMotorStartPower(pwmB);

    uint16_t pwmAValue = (uint16_t)((adjustedPwmA / 100.0f) * MAXIMUM_LEVEL);
    uint16_t pwmBValue = (uint16_t)((adjustedPwmB / 100.0f) * MAXIMUM_LEVEL);

    pwmAValue = (uint16_t)constrain(pwmAValue, 0, MAXIMUM_LEVEL);
    pwmBValue = (uint16_t)constrain(pwmBValue, 0, MAXIMUM_LEVEL);

    if (adjustedPwmA > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, pwmAValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwmAValue);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    } else if (adjustedPwmB > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, pwmBValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, pwmBValue);
    } else {
        stopMotors();
    }
}

void stopMotors() {
    pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
}

void resetPid() {
    previousError = 0.0f;
    integral = 0.0f;
}

void clearCommandBuffer() {
    for (int i = 0; i < 20; i++) {
        commandBuffer[i] = '\0';
    }

    commandIndex = 0;
    hasPendingCommand = false;
}

void processCommand() {
    commandBuffer[commandIndex] = '\0';

    if (commandIndex == 0) {
        clearCommandBuffer();
        return;
    }

    if (strcmp(commandBuffer, "start") == 0) {
        resetPid();
        pidRunning = true;
        printf("\nPID started\n");
    } else if (strcmp(commandBuffer, "stop") == 0) {
        pidRunning = false;
        resetPid();
        stopMotors();
        printf("\nPID stopped, motors off\n");
    } else {
        printf("\nUnknown command: %s\n", commandBuffer);
        printf("Use 'start' or 'stop'\n");
    }

    clearCommandBuffer();

    printf("Command: ");
    fflush(stdout);
}

void handleSerialCommands() {
    int ch = getchar_timeout_us(0);

    if (ch == PICO_ERROR_TIMEOUT) {
        if (hasPendingCommand) {
            absolute_time_t now = get_absolute_time();
            int64_t time_since_command = absolute_time_diff_us(lastCommandTime, now);

            if (time_since_command > 500000) {
                processCommand();
            }
        }

        return;
    }

    if (ch == '\r' || ch == '\n') {
        processCommand();
        return;
    }

    if (ch == 8 || ch == 127) {
        if (commandIndex > 0) {
            commandIndex--;
            commandBuffer[commandIndex] = '\0';
        }

        hasPendingCommand = true;
        lastCommandTime = get_absolute_time();
        return;
    }

    if (commandIndex < 19) {
        commandBuffer[commandIndex] = (char)ch;
        commandIndex++;
        commandBuffer[commandIndex] = '\0';

        hasPendingCommand = true;
        lastCommandTime = get_absolute_time();

        printf("%c", ch);
        fflush(stdout);
    } else {
        printf("\nCommand too long, clearing input\n");
        clearCommandBuffer();
        printf("Command: ");
        fflush(stdout);
    }
}

void mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);

    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu6050_configure() {
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    *temp = (buffer[6] << 8) | buffer[7];

    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}