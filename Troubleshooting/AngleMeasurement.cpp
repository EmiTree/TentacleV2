#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

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
const float ROLL_OFFSET = 2.3f;
const float PITCH_OFFSET = 4.4f;

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

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Only angle test starting...\n");

    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    mpu6050_reset();
    mpu6050_configure();

    uint8_t who_am_i = 0;
    uint8_t who_reg = WHO_AM_I_REG;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &who_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);

    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        while (true) {
            printf("MPU6050 not found\n");
            sleep_ms(1000);
        }
    }

    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;

    float roll = 0.0f;
    float pitch = 0.0f;

    absolute_time_t last_time = get_absolute_time();

    printf("roll, pitch\n");

    while (true) {
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

        float roll_out = roll + ROLL_OFFSET;
        float pitch_out = pitch + PITCH_OFFSET;

        printf("%.2f, %.2f\n", roll_out, pitch_out);

        sleep_ms(50);
    }
}