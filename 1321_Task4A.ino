/*******************************************************
 * File: Self_Balancing_Robot.ino
 * Description:
 *   Firmware for a two-wheeled self-balancing robot
 *   using MPU6050 IMU, cascaded PID (Angle + Velocity),
 *   quadrature encoders, and differential drive motors.
 *
 * Control Strategy:
 *   - Inner loop: Angle PID (stability)
 *   - Outer loop: Velocity PID (position/velocity hold)
 *   - Friction compensation added for dead-zone removal
 *******************************************************/

#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

/*=====================================================
  OBJECT DECLARATIONS
=====================================================*/

/* MPU6050 IMU object using I2C */
MPU6050 mpu(Wire);

/* Bluetooth serial object (currently unused) */
SoftwareSerial BT(0, 1);

/*=====================================================
  MOTOR PIN DEFINITIONS
=====================================================*/

/* Left motor pins */
int motor1A = A3;          // Direction pin A
int motor1B = A2;          // Direction pin B
int enable1 = 6;           // PWM enable pin

/* Right motor pins */
int motor2A = 9;           // Direction pin A
int motor2B = 4;           // Direction pin B
int enable2 = 5;           // PWM enable pin

/*=====================================================
  PID GAIN CONSTANTS
=====================================================*/

/* Angle PID gains (inner loop) */
double Kp = 38;            // Proportional gain
double Ki = 130;           // Integral gain
double Kd = 1.2;           // Derivative gain

/* Velocity PID gains (outer loop) */
double Kp_Vel = 0.1;
double ki_Vel = 0.001;
double kd_Vel = 0.00037;

/* Static friction compensation (PWM units) */
int frictionOffset = 13;

/*=====================================================
  SENSOR CALIBRATION CONSTANTS
=====================================================*/

/* Manual angle bias correction (degrees) */
double AngleOffset = 0.3;

/* Gyroscope offsets */
const float GYRO_X_OFFSET = -0.0153;
const float GYRO_Y_OFFSET = -0.0153;
const float GYRO_Z_OFFSET = -0.0153;

/* Accelerometer offsets */
const float ACC_X_OFFSET  = 0.0;
const float ACC_Y_OFFSET  = -0.0;
const float ACC_Z_OFFSET  = -1.0;

/*=====================================================
  TARGET STATES (COMMAND VARIABLES)
=====================================================*/

/*
 * targetVelocity:
 *   Desired linear velocity derived from Bluetooth input.
 *   Units: encoder pulses per control interval
 *   Range: approx ±400
 */
double targetVelocity = 0;

/* Desired yaw rate for turning (PWM differential) */
double targetYaw_RATE = 0;

/* Reserved servo-related variables (currently unused) */
double targetServo_RATE = 0;
double targetServo_OPEN_RATE = 0;

/*=====================================================
  SYSTEM STATE VARIABLES
=====================================================*/

/*
 * currentVelocity:
 *   Estimated robot velocity using encoder counts.
 */
double currentVelocity = 0;

/*
 * targetAngle:
 *   Desired balancing angle.
 *   Slight backward tilt allows forward motion.
 */
double targetAngle = -2.42;

/*
 * currentAngle:
 *   Fused angle from MPU6050 (degrees)
 */
double currentAngle = 0;

/*=====================================================
  PID ERROR VARIABLES
=====================================================*/

/* Angle PID variables */
double error_Ang = 0;
double previousError_Ang = 0;
double integral_Ang = 0;

/* Velocity PID variables */
double error_Vel = 0;
double previousError_Vel = 0;
double integral_Vel = 0;

/*=====================================================
  CONTROL OUTPUT VARIABLES
=====================================================*/

/*
 * controlSignal:
 *   Output of angle PID controller
 */
double controlSignal = 0;

/*
 * controlSignal_Vel:
 *   Output of velocity PID controller
 */
double controlSignal_Vel = 0;

/*
 * controlSignal_abs:
 *   Combined control output applied to motors
 */
double controlSignal_abs = 0;

/*=====================================================
  COMPLEMENTARY FILTER VARIABLES
=====================================================*/

double accAngle = 0;       // Angle from accelerometer
double gyroRate = 0;       // Angular velocity from gyroscope
double dt = 0.01;          // Loop time (seconds)
double alpha = 0.995;      // Complementary filter coefficient

/*=====================================================
  CONTROL STEP SIZES (BLUETOOTH COMMANDS)
=====================================================*/

const double ANGLE_STEP = 0.04;
const double VEL_STEP = 400;
const double YAW_STEP_RATE = 0.8;
const double SERVO_STEP_RATE = 0.1;

/*=====================================================
  ENCODER PIN DEFINITIONS
=====================================================*/

#define encodPinAR 2
#define encodPinBR 3
#define encodPinAL 7
#define encodPinBL 8

/*
 * wheel_pulse_count_left/right:
 *   Accumulated encoder counts since last loop
 */
volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;

/*=====================================================
  TIME MANAGEMENT VARIABLES
=====================================================*/

unsigned long currentTime;
unsigned long previousTime;

/*=====================================================
  FUNCTION: setup
=====================================================*/
/*
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic:
 *   - Initializes serial, I2C, MPU6050
 *   - Configures motor pins and encoder interrupts
 * Example Call:
 *   setup();
 */
void setup() {
    Serial.begin(9600);
    Wire.begin();

    /* Initialize IMU */
    mpu.begin();
    mpu.setGyroOffsets(GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
    mpu.setAccOffsets(ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET);

    /* Configure motor pins */
    pinMode(motor1A, OUTPUT);
    pinMode(motor1B, OUTPUT);
    pinMode(enable1, OUTPUT);
    pinMode(motor2A, OUTPUT);
    pinMode(motor2B, OUTPUT);
    pinMode(enable2, OUTPUT);

    /* Configure encoder pins */
    pinMode(encodPinAR, INPUT_PULLUP);
    pinMode(encodPinBR, INPUT_PULLUP);
    pinMode(encodPinAL, INPUT_PULLUP);
    pinMode(encodPinBL, INPUT_PULLUP);

    /* Attach interrupt for right encoder */
    attachInterrupt(digitalPinToInterrupt(encodPinAR), ISR_rightA, CHANGE);

    previousTime = millis();
}

/*=====================================================
  FUNCTION: handleBluetoothInput
=====================================================*/
/*
 * Function Name: handleBluetoothInput
 * Input:
 *   Serial characters via Bluetooth
 * Output:
 *   Updates targetAngle, targetVelocity, yaw rate
 * Logic:
 *   Decodes character commands to control robot motion
 * Example Call:
 *   handleBluetoothInput();
 */
void handleBluetoothInput() {
    while (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case '+': targetAngle += ANGLE_STEP; break;
            case '-': targetAngle -= ANGLE_STEP; break;
            case 'F': targetVelocity += VEL_STEP; break;
            case 'B': targetVelocity -= VEL_STEP; break;
            case 'R': targetYaw_RATE = YAW_STEP_RATE; break;
            case 'L': targetYaw_RATE = -YAW_STEP_RATE; break;

            case 'A':   // Emergency stop
                targetVelocity = 0;
                targetYaw_RATE = 0;
                integral_Vel = 0;
                integral_Ang = 0;
                previousError_Vel = 0;
                previousError_Ang = 0;
                Serial.println("STOP: Balance Hold");
                break;
        }
    }
}

/*=====================================================
  FUNCTION: loop
=====================================================*/
/*
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic:
 *   - Reads sensors
 *   - Computes PID control
 *   - Drives motors
 * Example Call:
 *   loop();
 */
void loop() {

    currentTime = millis();

    if (currentTime - previousTime >= 10) {

        dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        handleBluetoothInput();
        mpu.update();

        /* Angle estimation */
        currentAngle = mpu.getAngleY() - AngleOffset;

        /* Velocity estimation from encoders */
        currentVelocity = (wheel_pulse_count_left - wheel_pulse_count_right) / 2.0;

        /* Angle PID */
        error_Ang = targetAngle - currentAngle;
        integral_Ang += error_Ang * dt;
        integral_Ang = constrain(integral_Ang, -50, 50);
        double derivative_Ang = (error_Ang - previousError_Ang) / dt;
        previousError_Ang = error_Ang;

        /* Velocity PID */
        error_Vel = targetVelocity - currentVelocity;
        integral_Vel += error_Vel * dt;
        integral_Vel = constrain(integral_Vel, -50, 50);
        double derivative_Vel = (error_Vel - previousError_Vel) / dt;
        previousError_Vel = error_Vel;

        /* PID outputs */
        controlSignal =
            (Kp * error_Ang) +
            (Ki * integral_Ang) +
            (Kd * derivative_Ang);

        controlSignal_Vel =
            (Kp_Vel * error_Vel) +
            (ki_Vel * integral_Vel) +
            (kd_Vel * derivative_Vel);

        controlSignal_abs = controlSignal + controlSignal_Vel;

        /* Friction compensation */
        if (controlSignal_abs > 0) controlSignal_abs += frictionOffset;
        else if (controlSignal_abs < 0) controlSignal_abs -= frictionOffset;

        /* Motor PWM calculation */
        int leftMotorSpeed  = constrain(abs(controlSignal_abs - targetYaw_RATE), 0, 170);
        int rightMotorSpeed = constrain(abs(controlSignal_abs + targetYaw_RATE), 0, 170);

        controlMotors(controlSignal_abs, leftMotorSpeed, rightMotorSpeed);
    }
}

/*=====================================================
  FUNCTION: controlMotors
=====================================================*/
/*
 * Function Name: controlMotors
 * Input:
 *   controlSignal - direction decision
 *   leftSpeed     - left motor PWM
 *   rightSpeed    - right motor PWM
 * Output: None
 * Logic:
 *   Sets motor direction and PWM speed
 * Example Call:
 *   controlMotors(signal, lPWM, rPWM);
 */
void controlMotors(double controlSignal, int leftSpeed, int rightSpeed) {

    if (controlSignal > 0) {   // Forward
        digitalWrite(motor1A, HIGH);
        digitalWrite(motor1B, LOW);
        digitalWrite(motor2A, HIGH);
        digitalWrite(motor2B, LOW);
    } else {                   // Backward
        digitalWrite(motor1A, LOW);
        digitalWrite(motor1B, HIGH);
        digitalWrite(motor2A, LOW);
        digitalWrite(motor2B, HIGH);
    }

    analogWrite(enable1, leftSpeed);
    analogWrite(enable2, rightSpeed);
}

/*=====================================================
  FUNCTION: ISR_rightA
=====================================================*/
/*
 * Function Name: ISR_rightA
 * Input: Interrupt on encoder channel A
 * Output: Updates right wheel count
 * Logic:
 *   Determines rotation direction using quadrature logic
 */
void ISR_rightA() {
    if (digitalRead(encodPinAR) == digitalRead(encodPinBR))
        wheel_pulse_count_right++;
    else
        wheel_pulse_count_right--;
}
