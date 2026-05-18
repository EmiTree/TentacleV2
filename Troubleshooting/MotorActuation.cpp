#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

const int PWM_TOP = 1000;
const float MAX_SPEED = 100.0f;

float speed = 0.0f;

char input_buffer[20];
int input_index = 0;

absolute_time_t last_print_time;
absolute_time_t last_input_time;

bool has_pending_input = false;

float clamp(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void setup_pwm_pin(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_TOP);

    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0);
}

void set_motor(float speed_percent) {
    speed_percent = clamp(speed_percent, -MAX_SPEED, MAX_SPEED);

    int pwm_value = (int)((speed_percent < 0 ? -speed_percent : speed_percent) / 100.0f * PWM_TOP);

    if (speed_percent > 0) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, pwm_value);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwm_value);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    } else if (speed_percent < 0) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, pwm_value);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, pwm_value);
    } else {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    }
}

void clear_input_buffer() {
    for (int i = 0; i < 20; i++) {
        input_buffer[i] = '\0';
    }

    input_index = 0;
    has_pending_input = false;
}

void process_input() {
    input_buffer[input_index] = '\0';

    if (input_index == 0) {
        clear_input_buffer();
        return;
    }

    float new_speed = 0.0f;

    if (sscanf(input_buffer, "%f", &new_speed) == 1) {
        speed = clamp(new_speed, -MAX_SPEED, MAX_SPEED);
        set_motor(speed);

        printf("\nReceived: %s\n", input_buffer);
        printf("Speed set to %.1f%%\n", speed);
    } else {
        printf("\nInvalid input: %s\n", input_buffer);
    }

    clear_input_buffer();

    printf("Enter speed (-100 to 100): ");
    fflush(stdout);
}

void handle_serial_input() {
    int ch = getchar_timeout_us(0);

    if (ch == PICO_ERROR_TIMEOUT) {
        return;
    }

    if (ch == '\r' || ch == '\n') {
        process_input();
        return;
    }

    if (ch == 8 || ch == 127) {
        if (input_index > 0) {
            input_index--;
            input_buffer[input_index] = '\0';
        }

        has_pending_input = true;
        last_input_time = get_absolute_time();
        return;
    }

    if (input_index < 19) {
        input_buffer[input_index] = (char)ch;
        input_index++;
        input_buffer[input_index] = '\0';

        has_pending_input = true;
        last_input_time = get_absolute_time();

        printf("%c", ch);
        fflush(stdout);
    } else {
        printf("\nInput too long, clearing input\n");
        clear_input_buffer();
        printf("Enter speed (-100 to 100): ");
        fflush(stdout);
    }
}

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    setup_pwm_pin(MOTOR_PIN_P_1);
    setup_pwm_pin(MOTOR_PIN_P_2);
    setup_pwm_pin(MOTOR_PIN_Q_1);
    setup_pwm_pin(MOTOR_PIN_Q_2);

    set_motor(0);
    clear_input_buffer();

    printf("Motor actuation test ready\n");
    printf("Type a speed from -100 to 100, then click Send.\n");
    printf("Examples: 25, 80, -40, 0\n\n");
    printf("Enter speed (-100 to 100): ");
    fflush(stdout);

    last_print_time = get_absolute_time();

    while (true) {
        handle_serial_input();

        if (has_pending_input) {
            absolute_time_t now = get_absolute_time();
            int64_t time_since_input = absolute_time_diff_us(last_input_time, now);

            if (time_since_input > 500000) {
                process_input();
            }
        }

        absolute_time_t now = get_absolute_time();
        int64_t time_since_print = absolute_time_diff_us(last_print_time, now);

        if (time_since_print > 3000000) {
            last_print_time = now;

            printf("\nCurrent speed: %.1f%%\n", speed);
            printf("Enter speed (-100 to 100): ");
            fflush(stdout);
        }

        sleep_ms(10);
    }
}