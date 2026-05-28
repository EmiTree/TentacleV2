#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "PIDController.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

float setpoint = 0.0f;

PIDController pid(5.0f, 0.01f, 0.5f);

bool pidRunning = false;

char commandBuffer[40];
int commandIndex = 0;
bool hasPendingCommand = false;
absolute_time_t lastCommandTime;
absolute_time_t lastStatusPrint;

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

float maxPidOutput = 60.0f;
float maxPwm = 100.0f;
float motorStartPwm = 20.0f;

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

#define ACCEL_SCALE_FACTOR_4G 8192.0f
#define GYRO_SCALE_FACTOR_250DPS 131.0f

#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS

#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

const float PI_VALUE = 3.14159265359f;

float constrainValue(float value, float minVal, float maxVal);
void setupMotorPWM(int pin);
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
void printSettings();

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

    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;

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
    printf("Commands: start, stop, setpoint 0, maxpidoutput 50, motorstartpwm 20, kp 5, ki 0.01, kd 0.5\n");
    printSettings();

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

        float angle = roll + 2.3f;

        float pValue = 0.0f;
        float iValue = 0.0f;
        float dValue = 0.0f;

        float pidOutput = 0.0f;
        float motorOutput = 0.0f;
        float pwmA = 0.0f;
        float pwmB = 0.0f;

        if (pidRunning) {
            pidOutput = pid.update(setpoint, angle, dt, pValue, iValue, dValue);

            convertPidToMotor(pidOutput, pwmA, pwmB, motorOutput);

            driveMotors(pwmA, pwmB);
        } else {
            stopMotors();
            pid.reset();
        }

        absolute_time_t now = get_absolute_time();
        int64_t time_since_status = absolute_time_diff_us(lastStatusPrint, now);

        if (time_since_status > 50000) {
            lastStatusPrint = now;

            printf("angle=%.2f | pid=%.2f | p=%.2f | i=%.2f | d=%.2f | motor=%.2f | pwmA=%.2f | pwmB=%.2f\n",
                   angle,
                   pidOutput,
                   pValue,
                   iValue,
                   dValue,
                   motorOutput,
                   pwmA,
                   pwmB);
        }

        sleep_ms(10);
    }

    return 0;
}

float constrainValue(float value, float minVal, float maxVal) {
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

void convertPidToMotor(float pidOutput, float &pwmA, float &pwmB, float &motorOutput) {
    float availablePwmRange = maxPwm - motorStartPwm;

    if (availablePwmRange < 0.0f) {
        availablePwmRange = 0.0f;
    }

    motorOutput = pidOutput / maxPidOutput * availablePwmRange;
    motorOutput = constrainValue(motorOutput, -availablePwmRange, availablePwmRange);

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

    float adjustedPwm = motorStartPwm + pwm;

    return constrainValue(adjustedPwm, 0.0f, maxPwm);
}

void driveMotors(float pwmA, float pwmB) {
    float adjustedPwmA = addMotorStartPower(pwmA);
    float adjustedPwmB = addMotorStartPower(pwmB);

    uint16_t pwmAValue = (uint16_t)((adjustedPwmA / 100.0f) * MAXIMUM_LEVEL);
    uint16_t pwmBValue = (uint16_t)((adjustedPwmB / 100.0f) * MAXIMUM_LEVEL);

    pwmAValue = (uint16_t)constrainValue(pwmAValue, 0, MAXIMUM_LEVEL);
    pwmBValue = (uint16_t)constrainValue(pwmBValue, 0, MAXIMUM_LEVEL);

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
    pid.reset();
}

void clearCommandBuffer() {
    for (int i = 0; i < 40; i++) {
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

    char command[20];
    float value = 0.0f;

    int parts = sscanf(commandBuffer, "%19s %f", command, &value);

    if (parts >= 1) {
        if (strcmp(command, "start") == 0) {
            resetPid();
            pidRunning = true;
            printf("\nPID started\n");
        } else if (strcmp(command, "stop") == 0) {
            pidRunning = false;
            resetPid();
            stopMotors();
            printf("\nPID stopped, motors off\n");
        } else if (strcmp(command, "setpoint") == 0 && parts == 2) {
            setpoint = value;
            resetPid();
            printf("\nsetpoint set to %.2f\n", setpoint);
        } else if (strcmp(command, "maxpidoutput") == 0 && parts == 2) {
            maxPidOutput = constrainValue(value, 1.0f, 1000.0f);
            resetPid();
            printf("\nmaxPidOutput set to %.2f\n", maxPidOutput);
        } else if (strcmp(command, "motorstartpwm") == 0 && parts == 2) {
            motorStartPwm = constrainValue(value, 0.0f, maxPwm);
            resetPid();
            printf("\nmotorStartPwm set to %.2f\n", motorStartPwm);
        } else if (strcmp(command, "kp") == 0 && parts == 2) {
            pid.setKp(value);
            resetPid();
            printf("\nKp set to %.4f\n", value);
        } else if (strcmp(command, "ki") == 0 && parts == 2) {
            pid.setKi(value);
            resetPid();
            printf("\nKi set to %.4f\n", value);
        } else if (strcmp(command, "kd") == 0 && parts == 2) {
            pid.setKd(value);
            resetPid();
            printf("\nKd set to %.4f\n", value);
        } else if (strcmp(command, "settings") == 0) {
            printSettings();
        } else if (strcmp(command, "stopmotors") == 0) {
            pidRunning = false;
            resetPid();
            stopMotors();
            printf("\nFORCE STOP: motors off\n");
        } else {
            printf("\nUnknown command: %s\n", commandBuffer);
            printf("Use: start, stop, setpoint 0, maxpidoutput 50, motorstartpwm 20, kp 5, ki 0.01, kd 0.5, settings\n");
        } 
    }

    clearCommandBuffer();
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

    if (commandIndex < 39) {
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
    }
}

void printSettings() {
    printf("\nSettings:\n");
    printf("setpoint = %.2f\n", setpoint);
    printf("kp = %.4f\n", pid.getKp());
    printf("ki = %.4f\n", pid.getKi());
    printf("kd = %.4f\n", pid.getKd());
    printf("maxPidOutput = %.2f\n", maxPidOutput);
    printf("motorStartPwm = %.2f\n", motorStartPwm);
    printf("maxPwm = %.2f\n\n", maxPwm);
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