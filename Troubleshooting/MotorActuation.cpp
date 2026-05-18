#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

#define MAXIMUM_LEVEL 1000

const float MAX_PWM_PERCENT = 80.0f;
const float STEP_PERCENT = 5.0f;

float constrainFloat(float value, float minVal, float maxVal) {
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

void setMotorSpeed(float speedPercent) {
    speedPercent = constrainFloat(speedPercent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);

    uint16_t pwmValue = (uint16_t)((fabsf(speedPercent) / 100.0f) * MAXIMUM_LEVEL);

    if (speedPercent > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, pwmValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwmValue);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    } else if (speedPercent < 0.0f) {
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

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    setupMotorPWM(MOTOR_PIN_P_1);
    setupMotorPWM(MOTOR_PIN_P_2);
    setupMotorPWM(MOTOR_PIN_Q_1);
    setupMotorPWM(MOTOR_PIN_Q_2);

    float speedPercent = 0.0f;
    setMotorSpeed(speedPercent);

    printf("Manual motor test ready.\n");
    printf("Press + to increase, - to decrease, s or space to stop.\n");

    while (true) {
        int ch = getchar_timeout_us(0);

        if (ch == '+') {
            speedPercent += STEP_PERCENT;
        } else if (ch == '-') {
            speedPercent -= STEP_PERCENT;
        } else if (ch == 's' || ch == 'S' || ch == ' ') {
            speedPercent = 0.0f;
        }

        speedPercent = constrainFloat(speedPercent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);
        setMotorSpeed(speedPercent);

        printf("motor speed = %.1f%%\n", speedPercent);
        sleep_ms(100);
    }
}