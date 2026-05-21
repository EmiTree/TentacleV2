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

float roll = 0.0f;
float pitch = 0.0f;

void mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
}

uint8_t mpu6050_read_register(uint8_t reg) {
    uint8_t value = 0;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &value, 1, false);

    return value;
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);
}

bool mpu6050_begin() {
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(100);

    mpu6050_write_register(REG_PWR_MGMT_1, 0x80);
    sleep_ms(200);

    mpu6050_write_register(REG_PWR_MGMT_1, 0x00);
    sleep_ms(200);

    uint8_t who_am_i = mpu6050_read_register(WHO_AM_I_REG);

    printf("WHO_AM_I = 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        return false;
    }

    mpu6050_write_register(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE);
    mpu6050_write_register(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE);
    mpu6050_write_register(REG_SMPLRT_DIV, SAMPLE_RATE_DIV);

    return true;
}

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Angle measurement test starting...\n");

    bool ok = mpu6050_begin();

    if (!ok) {
        while (true) {
            printf("MPU6050 not found. Check wiring: SDA=GP4, SCL=GP5, VCC, GND.\n");
            sleep_ms(1000);
        }
    }

    printf("MPU6050 connected.\n");
    printf("roll,pitch\n");

    int16_t accel[3];
    int16_t gyro[3];

    absolute_time_t last_time = get_absolute_time();

    while (true) {
        printf("we're in the loop\n");
        mpu6050_read_raw(accel, gyro);

        float ax = accel[0] / ACCEL_SCALE_FACTOR;
        float ay = accel[1] / ACCEL_SCALE_FACTOR;
        float az = accel[2] / ACCEL_SCALE_FACTOR;

        float gx = gyro[0] / GYRO_SCALE_FACTOR;
        float gy = gyro[1] / GYRO_SCALE_FACTOR;

        absolute_time_t now = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;
        last_time = now;

        float roll_acc = atan2f(ay, az) * 180.0f / PI_VALUE;
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

        roll += gx * dt;
        pitch += gy * dt;

        roll = 0.98f * roll + 0.02f * roll_acc;
        pitch = 0.98f * pitch + 0.02f * pitch_acc;

        printf("%.2f,%.2f\n", roll, pitch);

        sleep_ms(50);
    }
}