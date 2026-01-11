#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

/* ============================================================
   OBJECT DECLARATIONS
   ============================================================ */

/*
 Variable Name: mpu
 Description  : MPU6050 object used to read accelerometer and gyroscope data
 Range        : N/A (object instance)
*/
MPU6050 mpu(Wire);

/*
 Variable Name: BT
 Description  : SoftwareSerial object for Bluetooth communication (HC-05)
 Range        : N/A (object instance)
*/
#define BT_RX 1
#define BT_TX 0
SoftwareSerial BT(BT_RX, BT_TX);

/* ============================================================
   MOTOR PIN DEFINITIONS
   ============================================================ */

/*
 Variable Name: motor1A, motor1B
 Description  : Direction control pins for left motor
 Range        : Digital HIGH / LOW
*/
int motor1A = A3;
int motor1B = A2;

/*
 Variable Name: enable1
 Description  : PWM enable pin for left motor
 Range        : 0–255 (PWM)
*/
int enable1 = 6;

/*
 Variable Name: motor2A, motor2B
 Description  : Direction control pins for right motor
 Range        : Digital HIGH / LOW
*/
int motor2A = 9;
int motor2B = 4;

/*
 Variable Name: enable2
 Description  : PWM enable pin for right motor
 Range        : 0–255 (PWM)
*/
int enable2 = 5;

/* ============================================================
   PID CONSTANTS
   ============================================================ */

/*
 Variable Name: Kp
 Description  : Proportional gain for angle error
 Range        : Typically 10–40
*/
double Kp = 22;

/*
 Variable Name: Ki
 Description  : Integral gain for accumulated angle error
 Range        : Typically 0–30
*/
double Ki = 20;

/*
 Variable Name: Kd
 Description  : Derivative gain for rate of change of error
 Range        : Typically 0–5
*/
double Kd = 1.2;

/* ============================================================
   SYSTEM STATE VARIABLES
   ============================================================ */

/*
 Variable Name: targetAngle
 Description  : Desired upright angle of robot
 Range        : -0.30 to +0.30 degrees
*/
double targetAngle = 0;

/*
 Variable Name: currentAngle
 Description  : Estimated tilt angle after sensor fusion
 Range        : Approximately -90 to +90 degrees
*/
double currentAngle = 0;

/*
 Variable Name: angularVelocity
 Description  : Angular velocity from gyroscope
 Range        : Degrees per second
*/
double angularVelocity = 0;

/*
 Variable Name: error
 Description  : Difference between targetAngle and currentAngle
 Range        : Depends on tilt
*/
double error = 0;

/*
 Variable Name: previousError
 Description  : Error from previous control cycle
 Range        : Same as error
*/
double previousError = 0;

/*
 Variable Name: integral
 Description  : Accumulated error over time (I-term)
 Range        : Limited to -50 to +50 to avoid wind-up
*/
double integral = 0;

/*
 Variable Name: controlSignal
 Description  : Final PID output before motor mapping
 Range        : Unbounded (later constrained)
*/
double controlSignal = 0;

/* ============================================================
   COMPLEMENTARY FILTER VARIABLES
   ============================================================ */

/*
 Variable Name: accAngle
 Description  : Angle calculated from accelerometer
 Range        : Degrees
*/
double accAngle = 0;

/*
 Variable Name: gyroRate
 Description  : Angular velocity from gyroscope
 Range        : Degrees per second
*/
double gyroRate = 0;

/*
 Variable Name: dt
 Description  : Time difference between control loops
 Range        : Seconds (≈0.005–0.02)
*/
double dt = 0.01;

/*
 Variable Name: alpha
 Description  : Complementary filter coefficient
 Range        : 0.95–0.995
*/
double alpha = 0.995;

/*
 Variable Name: ANGLE_STEP
 Description  : Step size for Bluetooth angle adjustment
 Range        : Degrees
*/
const double ANGLE_STEP = 0.04;

/*
 Variable Name: MAX_TARGET, MIN_TARGET
 Description  : Safety limits for target angle
 Range        : ±0.30 degrees
*/
const double MAX_TARGET = 0.30;
const double MIN_TARGET = -0.30;

/* ============================================================
   TIME VARIABLES
   ============================================================ */

/*
 Variable Name: currentTime
 Description  : Current system time in milliseconds
 Range        : Unsigned long
*/
unsigned long currentTime;

/*
 Variable Name: previousTime
 Description  : Time at previous loop iteration
 Range        : Unsigned long
*/
unsigned long previousTime;

/*
 Variable Name: deltaTime
 Description  : Time difference between loops (unused helper)
 Range        : Seconds
*/
double deltaTime;

/* ============================================================
   FRICTION OFFSET
   ============================================================ */

/*
 Variable Name: frictionOffset
 Description  : Minimum PWM added to overcome static friction
 Range        : 0–40 (depends on motor & surface)
*/
int frictionOffset = 20;

/* ============================================================
   FUNCTION: setup
   ============================================================ */
/*
 Function Name : setup
 Input         : None
 Output        : None
 Logic         :
   - Initializes Serial and I2C
   - Initializes MPU6050 and calibrates offsets
   - Configures motor control pins
   - Initializes time reference
 Example Call  : setup();
*/
void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (mpu.begin() != 0) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    mpu.calcOffsets();
    Serial.println("MPU6050 initialized!");

    pinMode(motor1A, OUTPUT);
    pinMode(motor1B, OUTPUT);
    pinMode(enable1, OUTPUT);
    pinMode(motor2A, OUTPUT);
    pinMode(motor2B, OUTPUT);
    pinMode(enable2, OUTPUT);

    Serial.println("Balancing bot ready!");
    previousTime = millis();
}

/* ============================================================
   FUNCTION: loop
   ============================================================ */
/*
 Function Name : loop
 Input         : None
 Output        : None
 Logic         :
   - Computes loop time
   - Reads MPU6050 sensor data
   - Applies complementary filter
   - Computes PID control output
   - Applies friction compensation
   - Drives motors accordingly
 Example Call  : loop();
*/
void loop() {
    currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    mpu.update();

    accAngle = mpu.getAngleY();
    gyroRate = mpu.getGyroY();

    currentAngle = alpha * (currentAngle + gyroRate * dt)
                 + (1 - alpha) * accAngle;

    error = targetAngle - currentAngle;
    integral += error * dt;
    integral = constrain(integral, -50, 50);

    double derivative = (error - previousError) / dt;
    previousError = error;

    controlSignal = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (controlSignal > 0) controlSignal += frictionOffset;
    else if (controlSignal < 0) controlSignal -= frictionOffset;

    int pwmValue = constrain(abs(controlSignal), 0, 120);

    if (controlSignal > 0) {
        analogWrite(enable1, pwmValue);
        analogWrite(enable2, pwmValue);
        digitalWrite(motor1A, HIGH);
        digitalWrite(motor1B, LOW);
        digitalWrite(motor2A, HIGH);
        digitalWrite(motor2B, LOW);
    }
    else if (controlSignal < 0) {
        analogWrite(enable1, pwmValue);
        analogWrite(enable2, pwmValue);
        digitalWrite(motor1A, LOW);
        digitalWrite(motor1B, HIGH);
        digitalWrite(motor2A, LOW);
        digitalWrite(motor2B, HIGH);
    }
    else {
        analogWrite(enable1, 0);
        analogWrite(enable2, 0);
        digitalWrite(motor1A, LOW);
        digitalWrite(motor1B, LOW);
        digitalWrite(motor2A, LOW);
        digitalWrite(motor2B, LOW);
    }

    delay(5);
}
