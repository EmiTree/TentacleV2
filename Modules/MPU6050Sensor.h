#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdint.h>

class MPU6050Sensor {
public:
    MPU6050Sensor(i2c_inst_t *i2cPort, uint8_t address, uint sdaPin, uint sclPin);

    bool begin();
    bool update(float dt);
    bool isOk() const;

    float getRoll() const;
    float getPitch() const;
    float getAngle() const;
    float getGyroX() const;
    float getGyroY() const;

private:
    i2c_inst_t *i2cPort;
    uint8_t address;
    uint sdaPin;
    uint sclPin;

    bool ok;

    float roll;
    float pitch;
    float angle;
    float gyroX;
    float gyroY;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool reset();
    bool configure();
    bool readRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
};