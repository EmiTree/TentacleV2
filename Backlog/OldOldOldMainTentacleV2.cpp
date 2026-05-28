#include "pico/stdlib.h"      // Basic Raspberry Pi Pico functions: GPIO, sleep, time, USB serial, etc.
#include "hardware/i2c.h"     // Lets the Pico communicate with I2C devices like the MPU6050 sensor.
#include "hardware/pwm.h"     // Lets the Pico output PWM signals, used here to control motors.
#include "PIDController.h"    // Your own PIDController class from the Modules folder.

#include <stdio.h>            // Gives printf, sscanf, fflush, etc.
#include <stdint.h>           // Gives fixed-size integer types like int16_t and uint8_t.
#include <math.h>             // Gives math functions like atan2f and sqrtf.
#include <string.h>           // Gives string functions like strcmp.

float setpoint = 0.0f; //The target angle for the tentacle.
PIDController pid(10.0f, 0.0f, 0.01f); //Create the PID controller from function "PIDController" in Modules, These starting values are: pid(Kp, Ki, Kd)

/*
    This decides whether the PID controller is actively driving the motors.

    false means:
    - motors are stopped
    - PID memory is reset

    true means:
    - sensor angle is read
    - PID output is calculated
    - motors are driven based on that output
*/
bool pidRunning = false; 

/*
    These variables are used for commands typed over USB serial.

    commandBuffer stores the characters typed by the user.
    commandIndex says where the next typed character should go.
    hasPendingCommand means a command has been partly typed but not processed yet.
*/
char commandBuffer[40];
int commandIndex = 0;
bool hasPendingCommand = false;

/*
    These remember times.

    lastCommandTime is used so the program can process a command after a pause,
    even if the user did not press Enter. #BJD

    lastStatusPrint is used so status messages print about every 50 ms instead
    of every single loop.
*/
absolute_time_t lastCommandTime; //Used to track when the last command character was typed, so the program can process the command after a pause even if Enter is not pressed.
absolute_time_t lastStatusPrint; //Used to limit how often status is printed, in microseconds.

/*
    Motor control pins.
    P1 = GP21 = Linkerwiel naar voren
    P2 = GP22 = Linker naar achter
    Q1 = GP27 = Rechterwiel naar voren
    Q2  = GP26 = rechterwiel naar achter
*/
const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

/*
    Motor/PID output limits.
*/
float maxPidOutput = 60.0f; //The PID output value that corresponds to maximum motor power. This is used to scale the PID output to the PWM range.
float motorStartPwm = 20.0f; //The minimum PWM percentage to apply when the motor should be moving, to help overcome static friction. This is added on top of the PID output after scaling, so it does not affect the PID calculations.
float maxPwm = 100.0f; //The maximum PWM percentage that can be sent to the motors. 100 means full range (0 to MAXIMUM_LEVEL). This can be used to limit top speed.
//float motorStartPwm = 20.0f; //The minimum PWM percentage to apply when the motor should be moving, to help overcome static friction. This is added on top of the PID output after scaling, so it does not affect the PID calculations.

/*
    LED pins. Not used right now, but possible for debugging
*/
#define LED_GREEN 13
#define LED_RED 12

/*
    I2C setup for the MPU6050 sensor.
*/
#define I2C_PORT i2c0 //The I2C hardware block used to communicate with the MPU6050. The Pico has i2c0 and i2c1, but this code only uses i2c0.
#define MPU6050_ADDR 0x68 //The I2C address of the MPU6050 sensor. This is the default address when the AD0 pin is low. If AD0 is high, the address would be 0x69.

/*
    Servo/PWM setup.

    SERVO_PIN is configured for PWM later.
    MAXIMUM_LEVEL is the PWM wrap value. With a wrap of 1000, the PWM level
    can be set from 0 to 1000.
*/
#define SERVO_PIN 14 //The GPIO pin used for the servo PWM signal. In this code
#define MAXIMUM_LEVEL 1000 //#BJD The maximum PWM level corresponding to 100% duty cycle. This is used to convert from percentage to the actual PWM level value.

/*
    MPU6050 register addresses.
    Registers are small memory locations inside the sensor.
    The Pico writes to and reads from these addresses to configure the sensor
    and retrieve measurements.
*/
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

/*
    Sensor scale factors.
    The MPU6050 gives raw integer readings.
    These constants convert raw readings into human-friendly units.
    Accelerometer:
        With +/- 4g selected, 8192 raw units equals 1g.
    Gyroscope:
        With +/- 250 degrees/second selected, 131 raw units equals
        1 degree/second.
    #BJD
*/
#define ACCEL_SCALE_FACTOR_4G 8192.0f
#define GYRO_SCALE_FACTOR_250DPS 131.0f

#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS

/*
    Sensor configuration values.
    ACCEL_CONFIG_VALUE = 0x08 selects +/- 4g accelerometer range.
    GYRO_CONFIG_VALUE = 0x00 selects +/- 250 degrees/second gyro range.
    SAMPLE_RATE_DIV = 0 means use the fastest base sample rate.
    #BJD
*/
#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

/*
    Pi, used to convert radians to degrees.
    atan2f returns radians, but angles are easier to read as degrees.
*/
const float PI_VALUE = 3.14159265359f;

/*
    Function declarations.
    C++ needs to know that these functions exist before main() uses them.
    The actual function bodies are written later in the file.
*/
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
    /*
        Initialize standard input/output.

        On the Pico, this makes USB serial communication work so you can use
        printf and type commands from your computer.
    */
    stdio_init_all();

    /*
        Wait until the USB serial connection is ready.

        Without this, the Pico might start printing before the computer is
        listening, causing startup messages to disappear.
    */
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Starting...\n");

    /*
        Set up the LEDs as output pins.

        gpio_init prepares the pin.
        gpio_set_dir(..., GPIO_OUT) makes it an output instead of an input.
    */
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);

    /*
        Configure each motor pin as a PWM output.

        PWM lets the program control motor strength by rapidly switching the
        pin on and off. A higher duty cycle means more motor power.
    */
    setupMotorPWM(MOTOR_PIN_P_1);
    setupMotorPWM(MOTOR_PIN_P_2);
    setupMotorPWM(MOTOR_PIN_Q_1);
    setupMotorPWM(MOTOR_PIN_Q_2);
    stopMotors();

    printf("Successfully setup motors\n");

    /*
        Start I2C at 400 kHz.
        I2C is the communication protocol used to talk to the MPU6050.
    */
    i2c_init(I2C_PORT, 400 * 1000);

    /*
        GPIO 4 and GPIO 5 are assigned to the I2C hardware function.
        On this setup for the MPU6050 to Pico, the wiring is:
        - GPIO 4 is SDA
        - GPIO 5 is SCL
    */
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);

    /*
        Pull-ups are needed for I2C lines.
        I2C devices communicate by pulling lines low, so the lines need to be
        pulled high when nobody is pulling them down. #BJD
    */
    gpio_pull_up(4);
    gpio_pull_up(5);

    printf("Initialized I2C port\n");

    /*
        Reset and configure the MPU6050 sensor.
        Reset clears old state.
        Configure sets accelerometer range, gyro range, and sample rate.
    */
    mpu6050_reset();
    mpu6050_configure();

    printf("Configured MPU6050\n");

    /*
        WHO_AM_I is a sensor identity register.

        A working MPU6050 should return 0x68. This is a useful check to confirm
        the sensor is connected and responding.
    */
    uint8_t who_am_i = 0;
    uint8_t who_reg = WHO_AM_I_REG;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &who_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);

    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);

    /*
        Who am i, checks what the device it is, it should answer with 0x68
        If the sensor does not respond with 0x68, stop forever.

        This prevents the motors from running when the angle sensor is missing
        or wired incorrectly.
    */
    if (who_am_i != 0x68) {
        while (true) {
            printf("MPU6050 not found!\n");
            stopMotors();
            sleep_ms(1000);
        }
    }

    /*
        Arrays for raw sensor readings.
        accel[0], accel[1], accel[2] are X, Y, Z acceleration.
        gyro[0], gyro[1], gyro[2] are X, Y, Z rotation speed.
        temp stores the raw temperature value, although this code does not use it. #BJD
    */
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;

    /*
        Estimated orientation angles.
        roll and pitch are continuously updated using both accelerometer and
        gyroscope data. #BJD
    */
    float roll = 0.0f;
    float pitch = 0.0f;

    /*
        last_time is used to calculate dt, the time since the previous loop.
        The gyro measures degrees per second, so the program needs time elapsed
        to convert rotation speed into angle change. 
    */
    absolute_time_t last_time = get_absolute_time();

    /*
        Configure SERVO_PIN as PWM too. #BJD
    */
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN); //
    pwm_config config = pwm_get_default_config();

    /*
        Set PWM timing.

        clkdiv slows the PWM clock.
        wrap sets the maximum counter value. #BJD
    */
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);

    /*
        Prepare the serial command buffer so it starts empty.
    */
    clearCommandBuffer();

    printf("PID is stopped.\n");
    printf("Commands: start, stop, setpoint 0, maxpidoutput 50, motorstartpwm 20, kp 5, ki 0.01, kd 0.5\n");
    printSettings();

    lastStatusPrint = get_absolute_time();

    /*
        Main control loop.

        Embedded programs run until stopped. Each loop:
        1. checks for serial commands
        2. reads the sensor
        3. estimates the current angle
        4. updates the PID controller if enabled
        5. drives or stops the motors
        6. prints status occasionally
    */
    while (true) {
        handleSerialCommands();

        /*
            Read raw accelerometer, gyroscope, and temperature values.
        */
        mpu6050_read_raw(accel, gyro, &temp);

        /*
            Convert raw acceleration to g units.
            For example, ax = 1.0 means about 1g on the X axis.#BJD
        */
        float ax = accel[0] / ACCEL_SCALE_FACTOR;
        float ay = accel[1] / ACCEL_SCALE_FACTOR;
        float az = accel[2] / ACCEL_SCALE_FACTOR;

        /*
            Convert raw gyro readings to degrees per second.
        */
        float gx = gyro[0] / GYRO_SCALE_FACTOR;
        float gy = gyro[1] / GYRO_SCALE_FACTOR;

        /*
            Calculate dt, the time since the previous loop, in seconds.
        */
        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1000000.0f;
        last_time = current_time;

        /*
            Estimate roll and pitch from the accelerometer.

            The accelerometer can tell which way gravity points, which gives
            an angle estimate. This is stable over time but noisy when the robot
            is moving. #BJD
        */
        float roll_acc = atan2f(ay, az) * 180.0f / PI_VALUE;
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

        /*
            Estimate angle change from the gyro.

            Gyro values are degrees per second, so multiplying by dt gives
            degrees moved during this loop.
        */
        roll += gx * dt;
        pitch += gy * dt;

        /*
            Complementary filter.

            This combines:
            - mostly gyro data, which is smooth and responsive
            - a little accelerometer data, which corrects long-term drift

            0.98 means 98% gyro estimate.
            0.02 means 2% accelerometer correction.
        */
        roll = 0.98f * roll + 0.02f * roll_acc;
        pitch = 0.98f * pitch + 0.02f * pitch_acc;

        /*
            The controlled angle is based on roll.

            The +2.3 is probably a calibration offset. It shifts the measured
            angle so the physical "neutral" position reads closer to zero.
        */
        float angle = roll + 2.3f;

        /*
            These variables are filled in by the PID and motor conversion code.
            They are also printed so you can see what the controller is doing.
        */
        float pValue = 0.0f;
        float iValue = 0.0f;
        float dValue = 0.0f;

        float pidOutput = 0.0f;
        float motorOutput = 0.0f;
        float pwmA = 0.0f;
        float pwmB = 0.0f;

        /*
            If PID is enabled, calculate a correction and drive the motors.

             - The PID controller looks at the angle error (setpoint - angle) and
               calculates an output value to try to reduce that error.
             - The PID output is then converted into PWM values for the motors.
             - Finally, the motors are driven with those PWM values.

             If PID is disabled, make sure the motors are off and clear the PID
             memory so it does not build up old error while stopped.
        */
        if (pidRunning) {
            pidOutput = pid.update(setpoint, angle, dt, pValue, iValue, dValue);

            convertPidToMotor(pidOutput, pwmA, pwmB, motorOutput);

            driveMotors(pwmA, pwmB);
        } else {
            stopMotors();
            pid.reset();
        }

        /*
            Print status about every 50 ms.

            50000 microseconds = 50 milliseconds.
        */
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

        /*
            Small delay so the loop does not run too aggressively.
        */
        sleep_ms(10);
    }

    return 0;
}

/*
    Keep a value inside a minimum and maximum range.

    Example:
    constrainValue(120, 0, 100) returns 100.
    constrainValue(-5, 0, 100) returns 0.
    constrainValue(50, 0, 100) returns 50.
*/
float constrainValue(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/*
    Configure one GPIO pin for PWM motor control.
*/
void setupMotorPWM(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM); //use this pin as a pwm output pin

    uint slice_num = pwm_gpio_to_slice_num(pin); //#BJD
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f); //125 cause pico's default clock system 125 Mhz
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

/*
    Convert PID output into two motor direction PWM values.

    pidOutput can be positive or negative:
    - positive means drive in direction A
    - negative means drive in direction B
    - zero means no movement

    pwmA and pwmB are separated because motor drivers often use one input for
    one direction and another input for the opposite direction.
*/
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

/*
    Add minimum motor starting power.

    If the requested PWM is zero or negative, return zero.
    If the requested PWM is positive, add motorStartPwm so the motor gets enough
    power to actually start moving.
*/
float addMotorStartPower(float pwm) {
    if (pwm <= 0.0f) {
        return 0.0f;
    }

    float adjustedPwm = motorStartPwm + pwm;

    return constrainValue(adjustedPwm, 0.0f, maxPwm);
}

/*
    Send PWM values to the motor pins.

    Only one direction is driven at a time:
    - pwmA drives MOTOR_PIN_*_1
    - pwmB drives MOTOR_PIN_*_2

    This avoids telling the motor driver to drive both directions at once.
*/
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

/*
    Turn all motor PWM outputs off.
*/
void stopMotors() {
    pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
}

/*
    Clear the PID controller memory.

    This is useful when starting, stopping, or changing settings, because the
    old accumulated error may no longer be valid.
*/
void resetPid() {
    pid.reset();
}

/*
    Empty the command buffer.

    This removes all previously typed command characters and resets command
    tracking variables.
*/
void clearCommandBuffer() {
    for (int i = 0; i < 40; i++) {
        commandBuffer[i] = '\0';
    }

    commandIndex = 0;
    hasPendingCommand = false;
}

/*
    Interpret and run a command typed over USB serial.

    Commands look like:
    start
    stop
    setpoint 0
    kp 5
    ki 0.01
    kd 0.5
*/
void processCommand() {
    commandBuffer[commandIndex] = '\0';

    if (commandIndex == 0) {
        clearCommandBuffer();
        return;
    }

    char command[20];
    float value = 0.0f;

    /*
        sscanf splits the typed text into:
        - a command word
        - an optional number

        Example:
        "kp 5.0" gives command = "kp" and value = 5.0.
    */
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

/*
    Read characters typed over USB serial.

    This function is non-blocking, meaning it checks for input but does not
    freeze the whole control loop while waiting for the user to type.
*/
void handleSerialCommands() {
    int ch = getchar_timeout_us(0);

    /*
        No character was typed.

        If a command was partly typed and then the user paused for more than
        0.5 seconds, process it anyway.
    */
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

    /*
        Enter key means the command is complete.
    */
    if (ch == '\r' || ch == '\n') {
        processCommand();
        return;
    }

    /*
        Backspace support.

        ASCII 8 and 127 are common backspace/delete values depending on the
        terminal program.
    */
    if (ch == 8 || ch == 127) {
        if (commandIndex > 0) {
            commandIndex--;
            commandBuffer[commandIndex] = '\0';
        }

        hasPendingCommand = true;
        lastCommandTime = get_absolute_time();
        return;
    }

    /*
        Add normal typed characters to the command buffer.

        The buffer has 40 characters total, so only 39 are accepted. The last
        slot is saved for '\0', which marks the end of a C string.
    */
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

/*
    Print the current controller settings over USB serial.
*/
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

/*
    Reset and wake up the MPU6050.

    Writing 0x80 to REG_PWR_MGMT_1 resets the device.
    Writing 0x00 afterward wakes it up.
*/
void mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);

    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

/*
    Configure the MPU6050 measurement ranges and sample rate.
*/
void mpu6050_configure() {
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}

/*
    Read raw accelerometer, temperature, and gyroscope data from the MPU6050.

    The sensor returns 14 bytes:
    - 6 bytes accelerometer
    - 2 bytes temperature
    - 6 bytes gyroscope

    Each measurement is split into a high byte and a low byte, so the code
    combines two bytes into one signed 16-bit number.
*/
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