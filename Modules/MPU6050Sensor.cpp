#include "MPU6050Sensor.h"

#include <math.h>
#include <stdio.h>

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

#define ACCEL_SCALE_FACTOR_4G 8192.0f
#define GYRO_SCALE_FACTOR_250DPS 131.0f

#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

const float PI_VALUE = 3.14159265359f;
const uint I2C_TIMEOUT_US = 100000;

MPU6050Sensor::MPU6050Sensor(i2c_inst_t *i2cPort, uint8_t address, uint sdaPin, uint sclPin) {
    this->i2cPort = i2cPort;
    this->address = address;
    this->sdaPin = sdaPin;
    this->sclPin = sclPin;

    ok = false;

    roll = 0.0f;
    pitch = 0.0f;
    angle = 0.0f;
    gyroX = 0.0f;
    gyroY = 0.0f;
}

bool MPU6050Sensor::begin() {
    i2c_init(i2cPort, 100 * 1000);

    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);

    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    printf("Initialized I2C port\n");

    ok = reset();

    if (ok) {
        ok = configure();
    }

    if (ok) {
        uint8_t whoAmI = 0;
        bool whoOk = readRegister(WHO_AM_I_REG, &whoAmI);

        printf("MPU6050 WHO_AM_I readOk=%d value=0x%02X\n", whoOk, whoAmI);

        if (!whoOk || whoAmI != 0x68) {
            ok = false;
        }
    }

    if (ok) {
        printf("Configured MPU6050\n");
    } else {
        printf("MPU setup failed\n");
    }

    return ok;
}

bool MPU6050Sensor::update(float dt) {
    if (!ok) {
        return false;
    }

    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;

    bool readOk = readRaw(accel, gyro, &temp);

    if (!readOk) {
        ok = false;
        return false;
    }

    float ax = accel[0] / ACCEL_SCALE_FACTOR_4G;
    float ay = accel[1] / ACCEL_SCALE_FACTOR_4G;
    float az = accel[2] / ACCEL_SCALE_FACTOR_4G;

    gyroX = gyro[0] / GYRO_SCALE_FACTOR_250DPS;
    gyroY = gyro[1] / GYRO_SCALE_FACTOR_250DPS;

    float rollAcc = atan2f(ay, az) * 180.0f / PI_VALUE;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

    roll += gyroX * dt;
    pitch += gyroY * dt;

    roll = 0.98f * roll + 0.02f * rollAcc;
    pitch = 0.98f * pitch + 0.02f * pitchAcc;

    angle = roll + 2.3f;

    return true;
}

bool MPU6050Sensor::isOk() const {
    return ok;
}

float MPU6050Sensor::getRoll() const {
    return roll;
}

float MPU6050Sensor::getPitch() const {
    return pitch;
}

float MPU6050Sensor::getAngle() const {
    return angle;
}

float MPU6050Sensor::getGyroX() const {
    return gyroX;
}

float MPU6050Sensor::getGyroY() const {
    return gyroY;
}

bool MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};

    int result = i2c_write_timeout_us(
        i2cPort,
        address,
        data,
        2,
        false,
        I2C_TIMEOUT_US
    );

    return result == 2;
}

bool MPU6050Sensor::readRegister(uint8_t reg, uint8_t *value) {
    int writeResult = i2c_write_timeout_us(
        i2cPort,
        address,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        i2cPort,
        address,
        value,
        1,
        false,
        I2C_TIMEOUT_US
    );

    return readResult == 1;
}

bool MPU6050Sensor::reset() {
    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) {
        printf("MPU reset write failed\n");
        return false;
    }

    sleep_ms(200);

    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
        printf("MPU wake write failed\n");
        return false;
    }

    sleep_ms(200);

    return true;
}

bool MPU6050Sensor::configure() {
    if (!writeRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE)) {
        printf("MPU accel config write failed\n");
        return false;
    }

    if (!writeRegister(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE)) {
        printf("MPU gyro config write failed\n");
        return false;
    }

    if (!writeRegister(REG_SMPLRT_DIV, SAMPLE_RATE_DIV)) {
        printf("MPU sample rate write failed\n");
        return false;
    }

    return true;
}

bool MPU6050Sensor::readRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    int writeResult = i2c_write_timeout_us(
        i2cPort,
        address,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        i2cPort,
        address,
        buffer,
        14,
        false,
        I2C_TIMEOUT_US
    );

    if (readResult != 14) {
        return false;
    }

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    *temp = (buffer[6] << 8) | buffer[7];

    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];

    return true;
}