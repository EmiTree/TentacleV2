#include "pico/stdlib.h"      // Pico basics: GPIO pins, USB serial, time functions, sleep_ms().
#include "hardware/i2c.h"     // I2C communication, used to talk to the MPU6050 sensor.
#include "hardware/pwm.h"     // PWM output, used to control motor power.
#include "PIDController.h"    // Your own PID controller class from Modules/PIDController.
#include "ServoActuator.h"    // Your own servo control class from Modules/ServoActuator.
#include "MotorConverter.h"   // Your own motor scaling class from Modules/MotorConverter.

#include <stdio.h>            // printf() and sscanf().
#include <stdint.h>           // Fixed-size number types like int16_t and uint8_t.
#include <stdlib.h>           // atof(), which converts text to a float number.
#include <math.h>             // atan2f() and sqrtf(), used for angle calculation.
#include <string.h>           // strcmp() and strstr(), used to compare typed commands.

float setpoint = 0.0f; //The setpoint is the target angle for the PID controller.

//Inserting and setting Modules
PIDController pid(5.0f, 0.01f, 0.5f); // PID tuning constants: Kp, Ki, Kd.
MotorConverter motorConverter(60.0f, 80.0f, 20.0f); // Motor conversion settings: PID output limit, max PWM, motor start PWM.
ServoActuator servo(14, 15, 16, 17); // ServoActuator(servo1Pin, servo2Pin, servo3Pin, servo4Pin) GP14, GP15, GP16, GP17 are the servo signal pins for servos 1-4 respectively.


bool pidRunning = false; // Starts with PID off for safety. Type "start" to turn on PID and "stop" to turn it off.
bool mpuOk = false; // This becomes true if the MPU6050 is successfully initialized and read. If it stays false, PID will not run, but servo commands and settings commands still work.

/*
    Serial command input. 
*/
char commandBuffer[40];
int commandIndex = 0;
bool hasPendingCommand = false;
absolute_time_t lastCommandTime;

/*
    Motor driver pins.
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
    LED pins.
    Possible pins that show if the robot is in a safe region for balancing when starting the robot.
    This function was used for testing the MPU and is not yet implemented in the current version of the robot.
*/
#define LED_GREEN 13
#define LED_RED 12

/*
    I2C settings for the MPU6050.
    MPU is connected to i2c0.
    MPU6050 address is 0x68
*/
#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68

/*
    PWM resolution.

    Motor PWM values are written from 0 to MAXIMUM_LEVEL.
    With MAXIMUM_LEVEL = 1000:

        0    means 0%
        500  means 50%
        1000 means 100%
*/
#define MAXIMUM_LEVEL 1000

/*
    MPU6050 register addresses.
*/
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

/*
    Sensor scale factors.

    These convert raw MPU numbers into useful units:
        accelerometer -> g
        gyroscope     -> degrees per second
*/
#define ACCEL_SCALE_FACTOR_4G 8192.0f
#define GYRO_SCALE_FACTOR_250DPS 131.0f
#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS

/*
    Sensor configuration values.

    ACCEL_CONFIG_VALUE = 0x08:
        accelerometer range is +/- 4g

    GYRO_CONFIG_VALUE = 0x00:
        gyroscope range is +/- 250 degrees/second
*/
#define ACCEL_CONFIG_VALUE 0x08
#define GYRO_CONFIG_VALUE 0x00
#define SAMPLE_RATE_DIV 0

const float PI_VALUE = 3.14159265359f;

/*
    I2C timeout in microseconds.
    Normal blocking I2C can freeze forever if the sensor is unplugged.
    Timeout I2C returns false instead, so the program keeps responding.
*/
const uint I2C_TIMEOUT_US = 100000;

/*
    Last values for the "print" command.
    it saves the latest values here, and prints them only when you type:
        print
*/
float lastRoll = 0.0f;
float lastPitch = 0.0f;
float lastAngle = 0.0f;
float lastPidOutput = 0.0f;
float lastPValue = 0.0f;
float lastIValue = 0.0f;
float lastDValue = 0.0f;
float lastMotorOutput = 0.0f;
float lastPwmA = 0.0f;
float lastPwmB = 0.0f;

/*
These functions exist somewhere later in this file. Here are their names, what they return, and what inputs they need.
*/
float constrainValue(float value, float minVal, float maxVal);
void setupMotorPWM(int pin);
void driveMotors(float pwmA, float pwmB);
void stopMotors();
void resetPid();
void forceStopEverything();

bool mpu6050_write_register(uint8_t reg, uint8_t value);
bool mpu6050_read_register(uint8_t reg, uint8_t *value);
bool mpu6050_reset();
bool mpu6050_configure();
bool mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

void clearCommandBuffer();
void processCommand();
void handleSerialCommands();
void printSettings();
void printHelp();
void printLiveValues();

int main() {
    stdio_init_all(); // Start USB serial communication.

    /*
        Wait until the computer actually opens the serial connection.
        This helps to see the startup messages instead of missing them.
    */
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Starting...\n");

    /*
        Set up LEDs as output pins.
    */
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);

    /*
        Set up the four motor control pins as PWM outputs.
    */
    setupMotorPWM(MOTOR_PIN_P_1);
    setupMotorPWM(MOTOR_PIN_P_2);
    setupMotorPWM(MOTOR_PIN_Q_1);
    setupMotorPWM(MOTOR_PIN_Q_2);
    stopMotors();

    printf("Successfully setup motors\n");

    /*
        Start the servo module.
        This sets up the servo pins and prepares the servo failsafe logic.
    */
    servo.begin();

    /*
        Start I2C for the MPU6050.
        Set up the Pico so GP4 and GP5 become the I2C communication wires, run them at a slower reliable speed, 
        add pull-up behavior, and then print that setup is done.
        Normally I2C runs at 400 kHz, but that can be too fast for long wires or breadboard connections. 100 kHz can be more reliable
    */
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(4, GPIO_FUNC_I2C); // GP4 is SDA.
    gpio_set_function(5, GPIO_FUNC_I2C); // GP5 is SCL.

    gpio_pull_up(4); // I2C needs pull-up behavior.
    gpio_pull_up(5); // I2C needs pull-up behavior.

    printf("Initialized I2C port\n");

    /*
        Try to reset and configure the MPU.
        If this fails, mpuOk becomes false, and PID is not allowed to start.
    */
    mpuOk = mpu6050_reset();

    if (mpuOk) {
        mpuOk = mpu6050_configure();
    }

    /*
        WHO_AM_I is a sensor identity check.
        A working MPU6050 should answer 0x68.
    */
    if (mpuOk) {
        uint8_t who_am_i = 0;
        bool whoOk = mpu6050_read_register(WHO_AM_I_REG, &who_am_i);

        printf("MPU6050 WHO_AM_I readOk=%d value=0x%02X\n", whoOk, who_am_i);

        if (!whoOk || who_am_i != 0x68) {
            mpuOk = false;
        }
    }

    if (mpuOk) {
        printf("Configured MPU6050\n");
    } else {
        printf("MPU setup failed. PID will stay stopped, but servo and settings commands still work.\n");
    }

    /*
        this block prepares empty containers for sensor data, 
        starts angle variables at zero, records the starting time, clears old command input, and prints the starting status.
    */
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;

    float roll = 0.0f;
    float pitch = 0.0f;
    float angle = 0.0f;

    absolute_time_t last_time = get_absolute_time();

    clearCommandBuffer();

    printf("PID is stopped.\n");
    printf("Type help for all commands.\n");
    printSettings();

    /*
        Main loop.

        This loop runs forever.
        Every pass:
            1. checks serial commands
            2. updates servo logic
            3. reads the MPU if available
            4. calculates angle
            5. runs PID if enabled
            6. saves latest values for the print command
    */
    while (true) {
        handleSerialCommands(); // This checks if you typed a command, and if so, processes it. It also updates the hasPendingCommand variable and lastCommandTime for command timeout handling.
        servo.update(); // This updates the servo control logic, including timed moves and failsafes. Calling this every loop for check

        //reset to 0 for the current loop round. Then the PID/motor code may fill them in with actual numbers.
        float pValue = 0.0f;
        float iValue = 0.0f;
        float dValue = 0.0f;

        float pidOutput = 0.0f;
        float motorOutput = 0.0f;
        float pwmA = 0.0f;
        float pwmB = 0.0f;

        if (mpuOk) {
            bool readOk = mpu6050_read_raw(accel, gyro, &temp);

            if (!readOk) {
                /*
                    If the MPU suddenly fails, stop PID immediately.
                    This prevents the motors from running based on bad sensor
                    data.
                */
                mpuOk = false;
                pidRunning = false;
                resetPid();
                stopMotors();

                printf("\nMPU read failed. PID stopped and motors off.\n");
                printf("Check MPU wiring, power, or I2C noise. Commands still work.\n");
            } else {
                /*
                    Convert raw MPU values to physical units.
                */
                float ax = accel[0] / ACCEL_SCALE_FACTOR;
                float ay = accel[1] / ACCEL_SCALE_FACTOR;
                float az = accel[2] / ACCEL_SCALE_FACTOR;

                float gx = gyro[0] / GYRO_SCALE_FACTOR;
                float gy = gyro[1] / GYRO_SCALE_FACTOR;

                /*
                    dt is the time since the previous loop.
                    The gyro gives degrees per second, so we multiply by dt
                    to estimate how many degrees changed this loop.
                */
                absolute_time_t current_time = get_absolute_time();
                float dt = absolute_time_diff_us(last_time, current_time) / 1000000.0f;
                last_time = current_time;

                /*
                    Estimate roll and pitch from the accelerometer.
                    Accelerometer angle is stable over time, but can be noisy
                    during movement.
                */
                float roll_acc = atan2f(ay, az) * 180.0f / PI_VALUE;
                float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI_VALUE;

                /*
                    Estimate roll and pitch from the gyro.
                */
                roll += gx * dt;
                pitch += gy * dt;

                /*
                    Complementary filter.

                    98% gyro:
                        fast and smooth

                    2% accelerometer:
                        corrects long-term drift

                    Trust the gyro almost completely, but let the accelerometer gently correct it over time
                */
                roll = 0.98f * roll + 0.02f * roll_acc;
                pitch = 0.98f * pitch + 0.02f * pitch_acc;

                /*
                    This is the angle used by the PID.

                    Right now it uses roll plus a calibration offset.
                    If your physical motion is pitch instead, this can become:
                        angle = pitch + 2.3f;
                */
                angle = roll + 2.3f;

                if (pidRunning) {
                    /*
                        Run the PID.

                        Inputs:
                            setpoint = target angle
                            angle    = measured angle
                            dt       = loop time

                        Outputs:
                            pidOutput = total correction
                            p/i/d     = separate parts for debugging
                    */
                    pidOutput = pid.update(setpoint, angle, dt, pValue, iValue, dValue);

                    /*
                        Convert PID output into motor PWM. See MotorConverter class for details.
                        This module applies:
                            - PID output limit
                            - motor start PWM
                            - max PWM
                            - deadband
                            - response curve
                    */
                    MotorCommand motorCommand = motorConverter.convert(pidOutput);

                    motorOutput = motorCommand.motorOutput;
                    pwmA = motorCommand.pwmA;
                    pwmB = motorCommand.pwmB;

                    driveMotors(pwmA, pwmB);
                } else {
                    /*
                        If PID is not running, always keep motors off.

                        Resetting the PID prevents old integral/derivative
                        memory from affecting the next start.
                    */
                    stopMotors();
                    pid.reset();
                }
            }
        } else {
            /*
                If the MPU is not okay, PID cannot safely run.
            */
            pidRunning = false;
            stopMotors();
            pid.reset();
        }

        /*
            Save values for the print command.
        */
        lastRoll = roll;
        lastPitch = pitch;
        lastAngle = angle;
        lastPidOutput = pidOutput;
        lastPValue = pValue;
        lastIValue = iValue;
        lastDValue = dValue;
        lastMotorOutput = motorOutput;
        lastPwmA = pwmA;
        lastPwmB = pwmB;

        sleep_ms(10);
    }

    return 0;
}

//-------------start of functions that were declared above main() but defined after main()-----------

/*
    Keep a value inside a minimum and maximum range.
*/
float constrainValue(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/*
    Configure a motor pin as a PWM output.
*/
void setupMotorPWM(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

/*
    Write converted PWM values to the motor pins.

    pwmA means one direction.
    pwmB means the other direction.
*/
void driveMotors(float pwmA, float pwmB) {
    uint16_t pwmAValue = (uint16_t)((pwmA / 100.0f) * MAXIMUM_LEVEL);
    uint16_t pwmBValue = (uint16_t)((pwmB / 100.0f) * MAXIMUM_LEVEL);

    pwmAValue = (uint16_t)constrainValue(pwmAValue, 0, MAXIMUM_LEVEL);
    pwmBValue = (uint16_t)constrainValue(pwmBValue, 0, MAXIMUM_LEVEL);

    if (pwmA > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, pwmAValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwmAValue);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    } else if (pwmB > 0.0f) {
        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, pwmBValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, pwmBValue);
    } else {
        stopMotors();
    }
}

/*
    Turn all motor outputs off.
*/
void stopMotors() {
    pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);
    pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
    pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
}

void resetPid() {
    pid.reset();
}

/*
    Emergency stop for the whole system.
*/
void forceStopEverything() {
    pidRunning = false;
    resetPid();
    stopMotors();
    servo.stop();

    printf("\nFORCE STOP: PID off, motors off, servos stopped\n");
}

/*
    Write one value to one MPU6050 register using timeout I2C.
*/
bool mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};

    int result = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        data,
        2,
        false,
        I2C_TIMEOUT_US
    );

    return result == 2;
}

/*
    Read one MPU6050 register using timeout I2C.
*/
bool mpu6050_read_register(uint8_t reg, uint8_t *value) {
    int writeResult = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        value,
        1,
        false,
        I2C_TIMEOUT_US
    );

    return readResult == 1;
}

bool mpu6050_reset() {
    if (!mpu6050_write_register(REG_PWR_MGMT_1, 0x80)) {
        printf("MPU reset write failed\n");
        return false;
    }

    sleep_ms(200);

    if (!mpu6050_write_register(REG_PWR_MGMT_1, 0x00)) {
        printf("MPU wake write failed\n");
        return false;
    }

    sleep_ms(200);

    return true;
}

bool mpu6050_configure() {
    if (!mpu6050_write_register(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE)) {
        printf("MPU accel config write failed\n");
        return false;
    }

    if (!mpu6050_write_register(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE)) {
        printf("MPU gyro config write failed\n");
        return false;
    }

    if (!mpu6050_write_register(REG_SMPLRT_DIV, SAMPLE_RATE_DIV)) {
        printf("MPU sample rate write failed\n");
        return false;
    }

    return true;
}

/*
    Read 14 bytes from the MPU:
        6 accel bytes
        2 temperature bytes
        6 gyro bytes
*/
bool mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    int writeResult = i2c_write_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (writeResult != 1) {
        return false;
    }

    int readResult = i2c_read_timeout_us(
        I2C_PORT,
        MPU6050_ADDR,
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

/*
    Clear the typed command.
*/
void clearCommandBuffer() {
    for (int i = 0; i < 40; i++) {
        commandBuffer[i] = '\0';
    }

    commandIndex = 0;
    hasPendingCommand = false;
}

/*
    Process one completed typed command. ------Start code for processing serial commands-------------
*/
void processCommand() {
    commandBuffer[commandIndex] = '\0';

    if (commandIndex == 0) { // Empty command, just ignore.
        clearCommandBuffer();
        return;
    }

    printf("\nReceived command: %s\n", commandBuffer); // Echo the command back for confirmation.

    if (strcmp(commandBuffer, "FORCE") == 0) { // Emergency stop command.
        forceStopEverything();
        clearCommandBuffer();
        return;
    }

    // temporary variables to understand typed commands
    char command[20];  // The main command, like "pid", "servo", "motor", "help", etc.
    char subCommand[20]; // The subcommand, like "start", "stop", "angle", "s1", etc. This is optional and may be empty for simple commands.
    float value = 0.0f; // A numeric value that some commands use, like a setpoint or a speed. This is optional and may be zero for commands that don't use it.

    /*
        Split typed input into:
            command
            subCommand
            value

        Example:
            "motor curve 1.5"

        becomes:
            command = "motor"
            subCommand = "curve"
            value = 1.5
    */
    int parts = sscanf(commandBuffer, "%19s %19s %f", command, subCommand, &value); // Read typed command and try to extract up to three parts: a main command, a subcommand, and a numeric value. The number of parts successfully read is stored in 'parts'.

    if (parts >= 1) {
        if (strcmp(command, "help") == 0) { // If the main command is "help", print the help text.
            printHelp();

        } else if (strcmp(command, "settings") == 0) { // If the main command is "settings", print the current settings for PID, motor converter, and servo.
            printSettings();

        } else if (strcmp(command, "print") == 0) { // If the main command is "print", print the latest sensor and PID values.
            printLiveValues();

        /*
            -----PID command group.----

            Examples:
                pid start
                pid stop
                pid set 0
        */
        } else if (strcmp(command, "pid") == 0 && parts >= 2) {
            if (strcmp(subCommand, "start") == 0) { // If the subcommand is "start", try to start the PID. If the MPU is not okay, print a warning and do not start PID.
                if (!mpuOk) {
                    printf("\nPID cannot start because MPU is not OK\n");
                } else {
                    resetPid();
                    pidRunning = true;
                    printf("\nPID started\n");
                }
            } else if (strcmp(subCommand, "stop") == 0) { // If the subcommand is "stop", stop the PID and turn off motors immediately.
                pidRunning = false;
                resetPid();
                stopMotors();
                printf("\nPID stopped, motors off\n");
            } else if ((strcmp(subCommand, "setpoint") == 0 || strcmp(subCommand, "set") == 0) && parts == 3) { // If the subcommand is "setpoint" or "set", and a numeric value is provided, update the PID setpoint to that value.
                setpoint = value;
                resetPid();
                printf("\nsetpoint set to %.2f\n", setpoint);
            } else if (strcmp(subCommand, "settings") == 0) { // If the subcommand is "settings", print the current PID settings.
                printSettings();
            } else {
                printf("\nUnknown PID command: %s\n", commandBuffer); // If the subcommand is not recognized, print an error and show the help text.
                printHelp();
            }

        /*
            -----Servo command group.------

            These commands are handled by the ServoActuator module.
        */
        } else if (strcmp(command, "servo") == 0 && parts >= 2) { 
            if (strcmp(subCommand, "angle") == 0 && parts == 3) { // If the subcommand is "angle" and a numeric value is provided, move the servo to that absolute angle.
                servo.moveToAngle(value);
            } else if (strcmp(subCommand, "move") == 0 && parts == 3) { // If the subcommand is "move" and a numeric value is provided, move the servo by that relative angle difference.
                servo.moveByAngle(value);
            } else if (strcmp(subCommand, "zero") == 0) { // If the subcommand is "zero", set the current angle as the new zero reference point for the servo.
                servo.zeroAngleHere();
            } else if (strcmp(subCommand, "stop") == 0 || strcmp(subCommand, "0") == 0) { // If the subcommand is "stop" or "0", stop all servo movement immediately.
                servo.stop();
            } else if (strcmp(subCommand, "status") == 0) { // If the subcommand is "status", print the current status of the servo, including estimated angles and any active failsafes.
                servo.printStatus();
            } else if (strcmp(subCommand, "help") == 0) { // If the subcommand is "help", print the help text for servo commands.
                servo.printHelp();
            } else if (strcmp(subCommand, "s1") == 0 && parts == 3) { // If the subcommand is "s1" and a numeric value is provided, set the speed of servo 1 to that value (as a percentage).
                servo.setServo1Speed(value);
            } else if (strcmp(subCommand, "s2") == 0 && parts == 3) { // If the subcommand is "s2" and a numeric value is provided, set the speed of servo 2 to that value (as a percentage).
                servo.setServo2Speed(-value);
            } else if (strcmp(subCommand, "s3") == 0 && parts == 3) { // If the subcommand is "s3" and a numeric value is provided, set the speed of servo 3 to that value (as a percentage).
                servo.setServo3Speed(value);
            } else if (strcmp(subCommand, "s4") == 0 && parts == 3) { // If the subcommand is "s4" and a numeric value is provided, set the speed of servo 4 to that value (as a percentage).
                servo.setServo4Speed(-value);
            } else if (subCommand[0] == '+' || subCommand[0] == '-' || (subCommand[0] >= '0' && subCommand[0] <= '9')) { // If the subcommand starts with a +, -, or a digit, and a numeric value is provided, set the speed of all servos to that value (as a percentage).
                servo.setSpeed((float)atof(subCommand));
            } else if (strcmp(subCommand, "minangle") == 0 && parts == 3) {
                servo.setMinAngle(value);
            } else if (strcmp(subCommand, "maxangle") == 0 && parts == 3) {
                servo.setMaxAngle(value);    
            } else { // If the subcommand is not recognized, print an error and show the help text for servo commands.
                printf("\nUnknown servo command: %s\n", commandBuffer);
                printHelp();
            }

        /*
            ---------------Motor converter command group.------------------

            These change how PID output becomes motor PWM.
        */
        } else if (strcmp(command, "motor") == 0 && parts >= 2) {
            if (strcmp(subCommand, "settings") == 0) {
                motorConverter.printSettings();
            } else if (strcmp(subCommand, "pidlimit") == 0 && parts == 3) {
                motorConverter.setPidOutputLimit(value);
                resetPid();
                printf("\npidOutputLimit set to %.2f\n", motorConverter.getPidOutputLimit());
            } else if (strcmp(subCommand, "maxpwm") == 0 && parts == 3) {
                motorConverter.setMaxPwm(value);
                resetPid();
                printf("\nmaxPwm set to %.2f\n", motorConverter.getMaxPwm());
            } else if (strcmp(subCommand, "startpwm") == 0 && parts == 3) {
                motorConverter.setMotorStartPwm(value);
                resetPid();
                printf("\nmotorStartPwm set to %.2f\n", motorConverter.getMotorStartPwm());
            } else if (strcmp(subCommand, "deadband") == 0) {
                if (parts == 3) {
                    motorConverter.setDeadband(value);
                    motorConverter.setDeadbandEnabled(true);
                    printf("\nmotor deadband set to %.4f\n", motorConverter.getDeadband());
                } else if (strstr(commandBuffer, "off") != nullptr) {
                    motorConverter.setDeadbandEnabled(false);
                    printf("\nmotor deadband disabled\n");
                } else if (strstr(commandBuffer, "on") != nullptr) {
                    motorConverter.setDeadbandEnabled(true);
                    printf("\nmotor deadband enabled\n");
                }
                resetPid();
            } else if (strcmp(subCommand, "curve") == 0) {
                if (parts == 3) {
                    motorConverter.setResponseCurve(value);
                    motorConverter.setResponseCurveEnabled(true);
                    printf("\nmotor response curve set to %.4f\n", motorConverter.getResponseCurve());
                } else if (strstr(commandBuffer, "off") != nullptr) {
                    motorConverter.setResponseCurveEnabled(false);
                    motorConverter.setResponseCurve(1.0f);
                    printf("\nmotor response curve disabled\n");
                } else if (strstr(commandBuffer, "on") != nullptr) {
                    motorConverter.setResponseCurveEnabled(true);
                    printf("\nmotor response curve enabled\n");
                }
                resetPid();
            } else {
                printf("\nUnknown motor command: %s\n", commandBuffer);
                printHelp();
            }

        /*
            Old simple PID commands.

            These are kept so your older workflow still works.
        */
        } else if (strcmp(command, "start") == 0) {
            if (!mpuOk) {
                printf("\nPID cannot start because MPU is not OK\n");
            } else {
                resetPid();
                pidRunning = true;
                printf("\nPID started\n");
            }
        } else if (strcmp(command, "stop") == 0) {
            forceStopEverything();
        } else if ((strcmp(command, "setpoint") == 0 || strcmp(command, "set") == 0) && parts >= 2) {
            setpoint = (parts == 2) ? (float)atof(subCommand) : value;
            resetPid();
            printf("\nsetpoint set to %.2f\n", setpoint);
        } else if (strcmp(command, "maxpidoutput") == 0 && parts >= 2) {
            float typedValue = (parts == 2) ? (float)atof(subCommand) : value;
            motorConverter.setPidOutputLimit(typedValue);
            resetPid();
            printf("\npidOutputLimit set to %.2f\n", motorConverter.getPidOutputLimit());
        } else if (strcmp(command, "motorstartpwm") == 0 && parts >= 2) {
            float typedValue = (parts == 2) ? (float)atof(subCommand) : value;
            motorConverter.setMotorStartPwm(typedValue);
            resetPid();
            printf("\nmotorStartPwm set to %.2f\n", motorConverter.getMotorStartPwm());
        } else if (strcmp(command, "kp") == 0 && parts >= 2) {
            float typedValue = (parts == 2) ? (float)atof(subCommand) : value;
            pid.setKp(typedValue);
            resetPid();
            printf("\nKp set to %.4f\n", typedValue);
        } else if (strcmp(command, "ki") == 0 && parts >= 2) {
            float typedValue = (parts == 2) ? (float)atof(subCommand) : value;
            pid.setKi(typedValue);
            resetPid();
            printf("\nKi set to %.4f\n", typedValue);
        } else if (strcmp(command, "kd") == 0 && parts >= 2) {
            float typedValue = (parts == 2) ? (float)atof(subCommand) : value;
            pid.setKd(typedValue);
            resetPid();
            printf("\nKd set to %.4f\n", typedValue);
        } else if (strcmp(command, "stopmotors") == 0) {
            forceStopEverything();
        } else {
            printf("\nUnknown command: %s\n", commandBuffer);
            printHelp();
        }
    }

    clearCommandBuffer();
}

/*
    Read serial input one character at a time.
*/
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

/*
    Print all currently adjustable settings.
*/
void printSettings() {
    printf("\n--- All settings ---\n");
    printf("mpuOk = %d\n", mpuOk);
    printf("pidRunning = %d\n", pidRunning);
    printf("setpoint = %.2f\n", setpoint);
    printf("kp = %.4f\n", pid.getKp());
    printf("ki = %.4f\n", pid.getKi());
    printf("kd = %.4f\n", pid.getKd());

    motorConverter.printSettings();
    servo.printStatus();

    printf("--------------------\n");
}

/*
    Print the latest measured values once.
*/
void printLiveValues() {
    printf("\n--- Live values ---\n");
    printf("mpuOk = %d\n", mpuOk);
    printf("pidRunning = %d\n", pidRunning);
    printf("roll = %.2f\n", lastRoll);
    printf("pitch = %.2f\n", lastPitch);
    printf("angle=%.2f | pid=%.2f | p=%.2f | i=%.2f | d=%.2f | motor=%.2f | pwmA=%.2f | pwmB=%.2f\n",
           lastAngle,
           lastPidOutput,
           lastPValue,
           lastIValue,
           lastDValue,
           lastMotorOutput,
           lastPwmA,
           lastPwmB);
    printf("-------------------\n");
}

/*
    Print command list.
*/
void printHelp() {
    printf("\nCommands:\n");

    printf("\nSUPER FAILSAFE:\n");
    printf("FORCE                  -> immediately stop PID, motors, and servos\n");

    printf("\nGeneral:\n");
    printf("help                   -> show all commands\n");
    printf("settings               -> show all settings\n");
    printf("print                  -> print live PID/MPU/motor values once\n");

    printf("\nPID commands:\n");
    printf("pid start              -> start PID\n");
    printf("pid stop               -> stop PID and motors\n");
    printf("pid setpoint 0         -> set PID target angle\n");
    printf("pid set 0              -> same as pid setpoint 0\n");
    printf("pid settings           -> print all settings\n");

    printf("\nOld PID commands still work:\n");
    printf("start                  -> start PID\n");
    printf("stop                   -> stop PID, motors, and servos\n");
    printf("setpoint 0             -> set PID target angle\n");
    printf("set 0                  -> same as setpoint 0\n");
    printf("maxpidoutput 50        -> set PID output limit for motor scaling\n");
    printf("motorstartpwm 20       -> set motor start PWM\n");
    printf("kp 5                   -> set Kp\n");
    printf("ki 0.01                -> set Ki\n");
    printf("kd 0.5                 -> set Kd\n");
    printf("stopmotors             -> stop PID, motors, and servos\n");

    printf("\nMotor converter commands:\n");
    printf("motor settings         -> print motor converter settings\n");
    printf("motor pidlimit 60      -> PID output that maps to full motor power\n");
    printf("motor maxpwm 100       -> maximum motor PWM percent\n");
    printf("motor startpwm 20      -> minimum useful motor PWM percent\n");
    printf("motor deadband 0.03    -> ignore tiny PID outputs\n");
    printf("motor deadband on/off  -> enable or disable deadband\n");
    printf("motor curve 1.5        -> soften small corrections\n");
    printf("motor curve on/off     -> enable or disable response curve\n");

    printf("\nServo commands:\n");
    printf("servo angle 90         -> all servos go to estimated absolute +90 degrees\n");
    printf("servo move 90          -> all servos move +90 degrees from current position\n");
    printf("servo move -20         -> all servos move -20 degrees from current position\n");
    printf("servo zero             -> save all current servo positions as 0 degrees\n");
    printf("servo +20              -> rotate all servos at 20%% speed\n");
    printf("servo -20              -> rotate all servos the other direction\n");
    printf("servo s1 +25           -> set only servo 1 speed\n");
    printf("servo s2 -20           -> set only servo 2 speed\n");
    printf("servo s3 +25           -> set only servo 3 speed\n");
    printf("servo s4 -20           -> set only servo 4 speed\n");
    printf("servo stop             -> stop all servos\n");
    printf("servo status           -> print servo status\n");
    printf("servo help             -> print servo help from module\n");
    printf("servo minangle -360 -> set minimum allowed servo angle\n");
    printf("servo maxangle 500  -> set maximum allowed servo angle\n");

    printf("\n");
}