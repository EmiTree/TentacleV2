#include "pico/stdlib.h"      // Pico basics: GPIO pins, USB serial, time functions, sleep_ms().
#include "hardware/i2c.h"     // I2C communication, used to talk to the MPU6050 sensor.
#include "hardware/pwm.h"     // PWM output, used to control motor power.
#include "PIDController.h"    // Your own PID controller class from Modules/PIDController.
#include "MotorConverter.h"   // Your own motor scaling class from Modules/MotorConverter.
#include "MPU6050Sensor.h"  // Your own MPU6050 sensor class from Modules/MPU6050Sensor.
#include "MotorDriver.h"


#include <stdio.h>            // printf() and sscanf().
#include <stdlib.h>           // atof(), which converts text to a float number.
#include <string.h>           // strcmp() and strstr(), used to compare typed commands.

float setpoint = 0.0f; //The setpoint is the target angle for the PID controller.

//Inserting and setting Modules
PIDController pid(5.0f, 0.01f, 0.5f); // PID tuning constants: Kp, Ki, Kd.
MotorConverter motorConverter(60.0f, 80.0f, 20.0f); // Motor conversion settings: PID output limit, max PWM, motor start PWM.
MPU6050Sensor mpu(i2c0, 0x68, 4, 5); // MPU6050 sensor (i2cPort, address, sdaPin, sclPin)
MotorDriver motorDriver(21, 22, 27, 26); // MotorDriver(pinP1, pinP2, pinQ1, pinQ2) P1 = GP21 = Linkerwiel naar voren, P2 = GP22 = Linker naar achter, Q1 = GP27 = Rechterwiel naar voren, Q2  = GP26 = rechterwiel naar achter



bool pidRunning = false; // Starts with PID off for safety. Type "start" to turn on PID and "stop" to turn it off.
bool mpuOk = false; // This becomes true if the MPU6050 is successfully initialized and read. If it stays false, PID will not run

/*
    Serial command input. 
*/
char commandBuffer[40];
int commandIndex = 0;
bool hasPendingCommand = false;
absolute_time_t lastCommandTime;

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


void resetPid();
void forceStopEverything();

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
    Set up the motor driver pins as PWM outputs.
*/
motorDriver.begin();

printf("Successfully setup motors\n");

   mpuOk = mpu.begin();

    if (!mpuOk) {
        printf("MPU setup failed. PID will stay stopped, but settings commands still work.\n");
    } else {
        printf("MPU setup succeeded. PID can be started with the 'start' command.\n");
    }

    absolute_time_t last_time = get_absolute_time();

    clearCommandBuffer();

    printf("PID is stopped.\n");
    printf("Type help for all commands.\n");
    printSettings();


    while (true) {
        handleSerialCommands(); // This checks if you typed a command, and if so, processes it. It also updates the hasPendingCommand variable and lastCommandTime for command timeout handling.

        //reset to 0 for the current loop round. Then the PID/motor code may fill them in with actual numbers.
        float pValue = 0.0f;
        float iValue = 0.0f;
        float dValue = 0.0f;

        float pidOutput = 0.0f;
        float motorOutput = 0.0f;
        float pwmA = 0.0f;
        float pwmB = 0.0f;

        absolute_time_t current_time = get_absolute_time();
        float dt = absolute_time_diff_us(last_time, current_time) / 1000000.0f;
        last_time = current_time;

        if (mpu.update(dt)) {
            mpuOk = true;

            float angle = mpu.getAngle();
            float gx = mpu.getGyroX();

            if (pidRunning) {
                pidOutput = pid.update(setpoint, angle, gx, dt, pValue, iValue, dValue);

                MotorCommand motorCommand = motorConverter.convert(pidOutput);

                motorOutput = motorCommand.motorOutput;
                pwmA = motorCommand.pwmA;
                pwmB = motorCommand.pwmB;

                motorDriver.drive(pwmA, pwmB);
            } else {
                motorDriver.stop();
                pid.reset();
            }

            lastRoll = mpu.getRoll();
            lastPitch = mpu.getPitch();
            lastAngle = mpu.getAngle();
        } else {
            mpuOk = false;
            pidRunning = false;
            resetPid();
            motorDriver.stop();

            printf("\nMPU read failed. PID stopped and motors off.\n");
            printf("Check MPU wiring, power, or I2C noise. Commands still work.\n");
        }

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


void resetPid() {
    pid.reset();
}

/*
    Emergency stop for the whole system.
*/
void forceStopEverything() {
    pidRunning = false;
    resetPid();
    motorDriver.stop();

    printf("\nFORCE STOP: PID off, motors off\n");
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
    char command[20];  // The main command, like "pid", "motor", "help", etc.
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

        } else if (strcmp(command, "settings") == 0) { // If the main command is "settings", print the current settings for PID, motor converter
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
                motorDriver.stop();
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

    if (ch == '\r' || ch == '\n') { // If the character is a newline, process the command.
        processCommand();
        return;
    }

    if (ch == 8 || ch == 127) { // If the character is backspace, remove the last character from the command buffer.
        if (commandIndex > 0) {
            commandIndex--;
            commandBuffer[commandIndex] = '\0';
        }

        hasPendingCommand = true;
        lastCommandTime = get_absolute_time();
        return;
    }

    if (commandIndex < 39) { // If the character is a regular character and there is space in the buffer, add it to the command buffer.
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
    printf("FORCE                  -> immediately stop PID, motors\n");

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
    printf("stop                   -> stop PID, motors\n");
    printf("setpoint 0             -> set PID target angle\n");
    printf("set 0                  -> same as setpoint 0\n");
    printf("maxpidoutput 50        -> set PID output limit for motor scaling\n");
    printf("motorstartpwm 20       -> set motor start PWM\n");
    printf("kp 5                   -> set Kp\n");
    printf("ki 0.01                -> set Ki\n");
    printf("kd 0.5                 -> set Kd\n");
    printf("stopmotors             -> stop PID, motors\n");

    printf("\nMotor converter commands:\n");
    printf("motor settings         -> print motor converter settings\n");
    printf("motor pidlimit 60      -> PID output that maps to full motor power\n");
    printf("motor maxpwm 100       -> maximum motor PWM percent\n");
    printf("motor startpwm 20      -> minimum useful motor PWM percent\n");
    printf("motor deadband 0.03    -> ignore tiny PID outputs\n");
    printf("motor deadband on/off  -> enable or disable deadband\n");
    printf("motor curve 1.5        -> soften small corrections\n");
    printf("motor curve on/off     -> enable or disable response curve\n");
    printf("\n");
}