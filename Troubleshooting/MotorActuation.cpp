#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

const int PWM_TOP = 1000;
const float MAX_SPEED = 100.0f;

float speed = 0.0f;

int pwm_p_1 = 0;
int pwm_p_2 = 0;
int pwm_q_1 = 0;
int pwm_q_2 = 0;

char input_buffer[40];
int input_index = 0;

absolute_time_t last_print_time;
absolute_time_t last_input_time;

bool has_pending_input = false;

float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

int clamp_int(int value, int low, int high) {
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

    printf("Setup PWM pin GP%d | slice %u | start value 0\n", pin, slice);
}

void apply_pin_values() {
    pwm_p_1 = clamp_int(pwm_p_1, 0, PWM_TOP);
    pwm_p_2 = clamp_int(pwm_p_2, 0, PWM_TOP);
    pwm_q_1 = clamp_int(pwm_q_1, 0, PWM_TOP);
    pwm_q_2 = clamp_int(pwm_q_2, 0, PWM_TOP);

    pwm_set_gpio_level(MOTOR_PIN_P_1, pwm_p_1);
    pwm_set_gpio_level(MOTOR_PIN_P_2, pwm_p_2);
    pwm_set_gpio_level(MOTOR_PIN_Q_1, pwm_q_1);
    pwm_set_gpio_level(MOTOR_PIN_Q_2, pwm_q_2);
}

void print_motor_debug() {
    printf("\n--- Motor debug ---\n");
    printf("Speed mode value: %.1f%%\n", speed);
    printf("PWM_TOP: %d\n", PWM_TOP);

    printf("p1 | GP%d | value %d\n", MOTOR_PIN_P_1, pwm_p_1);
    printf("p2 | GP%d | value %d\n", MOTOR_PIN_P_2, pwm_p_2);
    printf("q1 | GP%d | value %d\n", MOTOR_PIN_Q_1, pwm_q_1);
    printf("q2 | GP%d | value %d\n", MOTOR_PIN_Q_2, pwm_q_2);

    printf("-------------------\n\n");
}

void set_motor_speed(float speed_percent) {
    speed = clamp_float(speed_percent, -MAX_SPEED, MAX_SPEED);

    int pwm_value = (int)((speed < 0 ? -speed : speed) / 100.0f * PWM_TOP);

    if (speed > 0) {
        pwm_p_1 = pwm_value;
        pwm_q_1 = pwm_value;

        pwm_p_2 = 0;
        pwm_q_2 = 0;
    } else if (speed < 0) {
        pwm_p_1 = 0;
        pwm_q_1 = 0;

        pwm_p_2 = pwm_value;
        pwm_q_2 = pwm_value;
    } else {
        pwm_p_1 = 0;
        pwm_q_1 = 0;
        pwm_p_2 = 0;
        pwm_q_2 = 0;
    }

    apply_pin_values();

    printf("\nSet speed to %.1f%%\n", speed);
    print_motor_debug();
}

void set_one_pin(const char *pin_name, int value) {
    value = clamp_int(value, 0, PWM_TOP);

    if (strcmp(pin_name, "p1") == 0) {
        pwm_p_1 = value;
    } else if (strcmp(pin_name, "p2") == 0) {
        pwm_p_2 = value;
    } else if (strcmp(pin_name, "q1") == 0) {
        pwm_q_1 = value;
    } else if (strcmp(pin_name, "q2") == 0) {
        pwm_q_2 = value;
    } else {
        printf("\nUnknown pin name: %s\n", pin_name);
        return;
    }

    apply_pin_values();

    printf("\nSet %s to %d\n", pin_name, value);
    print_motor_debug();
}

void set_all_pins(int value) {
    value = clamp_int(value, 0, PWM_TOP);

    pwm_p_1 = value;
    pwm_p_2 = value;
    pwm_q_1 = value;
    pwm_q_2 = value;

    apply_pin_values();

    printf("\nSet all pins to %d\n", value);
    print_motor_debug();
}

void stop_motor() {
    speed = 0.0f;

    pwm_p_1 = 0;
    pwm_p_2 = 0;
    pwm_q_1 = 0;
    pwm_q_2 = 0;

    apply_pin_values();

    printf("\nStopped motor\n");
    print_motor_debug();
}

void print_help() {
    printf("\nCommands:\n");
    printf("speed 50     -> normal speed mode, -100 to 100\n");
    printf("p1 300       -> set GP%d PWM value\n", MOTOR_PIN_P_1);
    printf("p2 300       -> set GP%d PWM value\n", MOTOR_PIN_P_2);
    printf("q1 300       -> set GP%d PWM value\n", MOTOR_PIN_Q_1);
    printf("q2 300       -> set GP%d PWM value\n", MOTOR_PIN_Q_2);
    printf("all 0        -> set all pins to same PWM value\n");
    printf("stop         -> set all pins to 0\n");
    printf("help         -> print this help text\n");
    printf("PWM range is 0 to %d\n\n", PWM_TOP);
}

void clear_input_buffer() {
    for (int i = 0; i < 40; i++) {
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

    char command[12];
    int value = 0;

    int parts = sscanf(input_buffer, "%11s %d", command, &value);

    if (parts >= 1) {
        if (strcmp(command, "speed") == 0 && parts == 2) {
            set_motor_speed((float)value);
        } else if (strcmp(command, "p1") == 0 && parts == 2) {
            set_one_pin("p1", value);
        } else if (strcmp(command, "p2") == 0 && parts == 2) {
            set_one_pin("p2", value);
        } else if (strcmp(command, "q1") == 0 && parts == 2) {
            set_one_pin("q1", value);
        } else if (strcmp(command, "q2") == 0 && parts == 2) {
            set_one_pin("q2", value);
        } else if (strcmp(command, "all") == 0 && parts == 2) {
            set_all_pins(value);
        } else if (strcmp(command, "stop") == 0) {
            stop_motor();
        } else if (strcmp(command, "help") == 0) {
            print_help();
        } else {
            printf("\nUnknown command: %s\n", input_buffer);
            print_help();
        }
    }

    clear_input_buffer();

    printf("Enter command: ");
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

    if (input_index < 39) {
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
        printf("Enter command: ");
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

    stop_motor();
    clear_input_buffer();

    printf("Motor actuation test ready\n");
    print_help();
    printf("Enter command: ");
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

        if (time_since_print > 5000000) {
            last_print_time = now;

            print_motor_debug();
            printf("Enter command: ");
            fflush(stdout);
        }

        sleep_ms(10);
    }
}