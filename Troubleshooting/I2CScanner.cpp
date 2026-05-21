#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdio.h>

/*
    Raspberry Pi Pico I2C scanner.

    This scans the I2C bus and prints every address that responds.

    Wiring for i2c0:
        SDA -> GP4
        SCL -> GP5

    For your MPU6050, the expected address is usually:
        0x68

    Sometimes it can be:
        0x69

    If nothing is found, check:
        - VCC
        - GND
        - SDA/SCL swapped
        - loose wires
        - whether the sensor board needs pull-up resistors
*/

#define I2C_PORT i2c0

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

/*
    Start at 0x08 and end at 0x77.

    These are the normal usable 7-bit I2C device addresses.
    Addresses below 0x08 and above 0x77 are reserved.
*/
const int FIRST_I2C_ADDRESS = 0x08;
const int LAST_I2C_ADDRESS = 0x77;

/*
    I2C timeout so the scanner does not freeze forever if the bus is stuck.
*/
const uint I2C_TIMEOUT_US = 10000;

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("\nPico I2C scanner starting...\n");

    /*
        Use 100 kHz for scanning.

        This is slower than 400 kHz, but more forgiving while debugging.
    */
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("Scanning i2c0 on SDA=GP%d, SCL=GP%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

    while (true) {
        int foundCount = 0;

        printf("\nScan start\n");

        for (int address = FIRST_I2C_ADDRESS; address <= LAST_I2C_ADDRESS; address++) {
            uint8_t dummy = 0;

            /*
                Try to read 1 byte from each address.

                If a device acknowledges the address, the function usually
                returns 1. If no device responds, it returns an error code.
            */
            int result = i2c_read_timeout_us(
                I2C_PORT,
                address,
                &dummy,
                1,
                false,
                I2C_TIMEOUT_US
            );

            if (result >= 0) {
                printf("Found device at 0x%02X\n", address);
                foundCount++;
            }
        }

        if (foundCount == 0) {
            printf("No I2C devices found\n");
        } else {
            printf("Found %d I2C device(s)\n", foundCount);
        }

        printf("Scan done. Waiting 2 seconds...\n");

        sleep_ms(2000);
    }

    return 0;
}