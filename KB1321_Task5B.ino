#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/* =====================================================
   ================= GLOBAL OBJECTS ====================
   ===================================================== */

/*
Variable Name : mpu
Description   : MPU6050 IMU object used for angle estimation (tilt + gyro data)
Range         : Internal object (library handles it)
Purpose       : Gives tilt angle (getAngleY) and yaw gyro rate (getGyroZ)
*/
MPU6050 mpu(Wire);


/* =====================================================
   ================= MOTOR PINS ========================
   ===================================================== */

/*
Variable Name : motor1A, motor1B
Description   : Direction pins for LEFT motor driver (H-Bridge)
Range         : Digital HIGH / LOW
Purpose       : Determines motor direction (forward/backward)
*/
int motor1A = A3;
int motor1B = A2;

/*
Variable Name : enable1
Description   : PWM speed control pin for LEFT motor
Range         : 0 to 255
Purpose       : Controls speed using analogWrite
*/
int enable1 = 6;

/*
Variable Name : motor2A, motor2B
Description   : Direction pins for RIGHT motor driver (H-Bridge)
Range         : Digital HIGH / LOW
Purpose       : Determines motor direction (forward/backward)
*/
int motor2A = 9;
int motor2B = 4;

/*
Variable Name : enable2
Description   : PWM speed control pin for RIGHT motor
Range         : 0 to 255
Purpose       : Controls speed using analogWrite
*/
int enable2 = 5;

/*
Variable Name : buzz
Description   : Buzzer pin
Range         : Digital output
Purpose       : Gives audio feedback for commands
*/
const int buzz = A1;


/* =====================================================
   ================= ANGLE PID GAINS ===================
   ===================================================== */

/*
Inner Loop PID (Angle Control)
Purpose : Keeps robot upright (balancing loop)
Notes   : This loop must run fast (high frequency)
*/
double Kp = 30;      // Proportional gain (stiffness)
double Ki = 210;     // Integral gain (fixes steady-state drift)
double Kd = 1;       // Derivative gain (damping)

/*
Variable Name : prev_right, prev_left
Description   : Previous encoder pulse values (used for delta computation)
Range         : Encoder pulse count (long integer)
Purpose       : Used for velocity estimation
*/
long prev_right = 0;
long prev_left  = 0;


/* =====================================================
   ============ VELOCITY PID GAINS (OUTER LOOP) =========
   ===================================================== */

/*
Outer Loop PID (Velocity Control)
Purpose : Converts velocity error into target angle command
Logic   : If robot is slow, tilt forward more, if too fast tilt backward
*/
double Kp_Vel = 0.00075;      // Proportional velocity gain
double ki_Vel = 0.000;        // Integral velocity gain (currently disabled)
double kd_Vel = 0.000062;     // Derivative velocity gain

/*
Variable Name : alpha1, alpha2
Description   : Low-pass filter coefficients for derivative smoothing
Range         : 0 to 1
Purpose       : Prevents noisy derivative spikes
*/
double alpha1 = 0.7;   // velocity derivative filter
double alpha2 = 0.7;   // angle derivative filter


/* =====================================================
   ================= SYSTEM PARAMETERS =================
   ===================================================== */

/*
Variable Name : frictionOffset
Description   : Adds extra PWM to overcome static friction
Range         : Usually 3 to 30
Purpose       : Motors need extra push to start moving
*/
int frictionOffset = 4;

/*
Variable Name : AngleOffset
Description   : IMU mounting offset correction (calibration)
Range         : Typically -5 to +5 degrees
Purpose       : Ensures upright position corresponds to 0° error
*/
double AngleOffset = 1.1;


/* =====================================================
   ================= TARGET VARIABLES ==================
   ===================================================== */

/*
Variable Name : targetVelocity
Description   : Desired forward/backward velocity
Range         : Encoder pulses per second
Purpose       : Set by Bluetooth commands (F/B)
*/
double targetVelocity = 0;

/*
Variable Name : targetYaw_RATE
Description   : Desired turning rate (yaw angular velocity)
Range         : Small values (deg/sec)
Purpose       : Set by Bluetooth commands (L/R)
*/
double targetYaw_RATE = 0;


/* =====================================================
   ================= MPU OFFSETS =======================
   ===================================================== */

/*
These offsets must be experimentally tuned.
Purpose : Removes sensor bias (drift)
*/
const float GYRO_X_OFFSET = -0.153;
const float GYRO_Y_OFFSET = -0.153;
const float GYRO_Z_OFFSET = -0.153;

const float ACC_X_OFFSET  =  0.0;
const float ACC_Y_OFFSET  =  0.0;
const float ACC_Z_OFFSET  = -1.0;


/* =====================================================
   ================= STATE VARIABLES ===================
   ===================================================== */

/*
Variable Name : currentAngle
Description   : Current tilt angle of robot (Y-axis angle)
Range         : -90 to +90 degrees
Purpose       : Used in balancing PID
*/
double currentAngle = 0;

/*
Variable Name : currentVelocity
Description   : Measured robot velocity from encoder pulses
Range         : pulses/sec
Purpose       : Used in outer velocity loop
*/
double currentVelocity = 0;


/* =====================================================
   ============== ANGLE PID STATE VARIABLES =============
   ===================================================== */

/*
Variable Name : error_Ang
Description   : Tilt error = targetAngle_cmd - currentAngle
Range         : Typically -15 to +15 degrees
*/
double error_Ang = 0;

/*
Variable Name : prevError_Ang
Description   : Previous tilt error (used for derivative)
*/
double prevError_Ang = 0;

/*
Variable Name : integral_Ang
Description   : Accumulated error for integral action
Range         : limited using constrain()
*/
double integral_Ang = 0;

/*
Variable Name : filteredD_Ang
Description   : Filtered derivative term for angle PID
Purpose       : Prevents jitter
*/
double filteredD_Ang = 0;

/*
Variable Name : filteredD_Pos
Description   : Filtered derivative for velocity PID
*/
double filteredD_Pos = 0;

/*
Variable Name : targetAngle_cmd
Description   : Angle command generated by velocity PID
Range         : -8 to +8 degrees
Purpose       : If velocity error exists, robot leans accordingly
*/
volatile double targetAngle_cmd = 0.0;


/* =====================================================
   ============ VELOCITY PID STATE VARIABLES ============
   ===================================================== */

/*
Variable Name : error_Vel
Description   : Velocity error = targetVelocity - currentVelocity
*/
double error_Vel = 0;

/*
Variable Name : prevError_Vel
Description   : Previous velocity error for derivative computation
*/
double prevError_Vel = 0;

/*
Variable Name : integral_Vel
Description   : Integral accumulation for velocity PID
*/
double integral_Vel = 0;


/* =====================================================
   ================= YAW PID (RATE LOOP) ===============
   ===================================================== */

/*
Yaw PID controls turning.
We measure yaw rate from gyro Z axis (deg/sec).
Then we apply differential motor PWM.
*/

double Kp_Yaw = 0.9;
double Ki_Yaw = 0;
double Kd_Yaw = 0;

double error_Yaw = 0;
double prevError_Yaw = 0;
double integral_Yaw = 0;
double filteredD_Yaw = 0;

double yawRateMeasured = 0;   // deg/sec from gyro
double yawCommand = 0;        // correction to motors (PWM difference)

double alphaYaw = 0.7;        // yaw derivative filter coefficient


/* =====================================================
   ================= CONTROL LIMITS ====================
   ===================================================== */

/*
Variable Name : VEL_STEP
Description   : Step size for velocity command increments
Range         : pulses/sec
*/
const double VEL_STEP = 80;

/*
Variable Name : YAW_STEP_RATE
Description   : Step size for yaw rate command increments
Range         : deg/sec
*/
const double YAW_STEP_RATE = 10;


/* =====================================================
   ================= TIME VARIABLES ====================
   ===================================================== */

/*
Variable Name : currentTime, previousTime
Description   : Used for dt computation in velocity loop
Range         : microseconds
*/
unsigned long currentTime, previousTime = 0;

/*
Variable Name : dt
Description   : Velocity loop time step
Range         : seconds
*/
double dt;

/*
Variable Name : currentTime1, previousTime1
Description   : Used for dt computation in angle loop
*/
unsigned long currentTime1, previousTime1 = 0;

/*
Variable Name : dt1
Description   : Angle loop time step
Range         : seconds
*/
double dt1;


/* =====================================================
   ================= BLUETOOTH =========================
   ===================================================== */

/*
BT_RX and BT_TX are defined but NOT used properly here
because you are reading Serial instead of BT.
Still kept for future expansion.
*/
#define BT_RX 0
#define BT_TX 1
SoftwareSerial BT(BT_RX, BT_TX);


/* =====================================================
   ================= ENCODERS ==========================
   ===================================================== */

/*
Right encoder pins use external interrupts (2,3).
Left encoder pins (7,8) are NOT interrupts in Arduino Nano.
So only right encoder is used currently.
*/
#define encodPinAR 2
#define encodPinBR 3
#define encodPinAL 7
#define encodPinBL 8

/*
Variable Name : wheel_pulse_count_right
Description   : Encoder pulse count for right wheel
Range         : long integer, increases/decreases continuously
Purpose       : Used for velocity measurement
Volatile      : Must be volatile because ISR updates it
*/
volatile long wheel_pulse_count_right = 0;


/* =====================================================
   ================= SERVOS ============================
   ===================================================== */

/*
Servo pins
Purpose : Control external actuators
*/
#define SERVO1_PIN 10
#define SERVO2_PIN 11

Servo servo1, servo2;

/*
Variable Name : SERVO_STEP_RATE1, SERVO_STEP_RATE2
Description   : Servo angles in degrees
Range         : 0 to 180
Purpose       : Updated using Bluetooth commands
*/
double SERVO_STEP_RATE1 = 0;
double SERVO_STEP_RATE2 = 0;

const int SERVO_MIN = 0;
const int SERVO_MAX = 180;


/* =====================================================
   ====================== SETUP ========================
   ===================================================== */

/*
Function Name : setup
Input         : None
Output        : None
Logic         :
  - Initializes serial, I2C
  - Initializes MPU6050 with offsets
  - Initializes motor pins as OUTPUT
  - Initializes encoder pins and interrupt
  - Attaches servos
Example Call  : Automatically called by Arduino
*/
void setup() {

  // Initialize Serial communication
  Serial.begin(9600);

  // Initialize I2C bus
  Wire.begin();

  // Initialize MPU sensor
  mpu.begin();

  // Apply gyro and accelerometer calibration offsets
  mpu.setGyroOffsets(GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
  mpu.setAccOffsets(ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET);

  // Motor pins as output
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(enable1, OUTPUT);

  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(enable2, OUTPUT);

  // Encoder pins input pullup (for stable logic)
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);

  // Attach interrupt to right encoder channel A
  attachInterrupt(digitalPinToInterrupt(encodPinAR), ISR_rightA, CHANGE);

  // Attach servo motors
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Set initial servo positions
  servo1.write(SERVO_STEP_RATE1);
  servo2.write(SERVO_STEP_RATE2);
}


/* =====================================================
   ================= BLUETOOTH HANDLER =================
   ===================================================== */

/*
Function Name : handleBluetoothInput
Input         : None (reads Serial buffer)
Output        : None
Logic         :
  - Reads command characters from Serial
  - Updates target velocity, yaw rate, servo positions
  - Supports calibration adjustment of AngleOffset
Commands:
  F : forward
  B : backward
  L : left turn
  R : right turn
  S : stop
  U/D : servo1 adjust
  Q/E : servo2 adjust
  + / - : angle offset tuning
  X : prints AngleOffset
  M : buzzer beep
Example Call  : handleBluetoothInput();
*/
void handleBluetoothInput() {

  // Process all available characters
  while (Serial.available()) {

    char cmd = Serial.read();

    switch (cmd) {

      // Adjust angle offset slightly
      case '+':
        AngleOffset += 0.02;
        break;

      case '-':
        AngleOffset -= 0.02;
        break;

      // Print current offset
      case 'X':
        Serial.print("Angle Offset : ");
        Serial.println(AngleOffset);
        break;

      // Forward command
      case 'F':
        targetVelocity += VEL_STEP;
        break;

      // Backward command (bigger negative step)
      case 'B':
        targetVelocity -= 320;
        break;

      // Turning commands
      case 'R':
        targetYaw_RATE = YAW_STEP_RATE;
        break;

      case 'L':
        targetYaw_RATE = -YAW_STEP_RATE;
        break;

      // Stop command
      case 'S':
        targetYaw_RATE = 0;
        targetVelocity = 0;
        break;

      // Servo1 controls
      case 'U':
        SERVO_STEP_RATE1 = constrain(SERVO_STEP_RATE1 - 5, SERVO_MIN, SERVO_MAX);
        servo1.write(SERVO_STEP_RATE1);
        break;

      case 'D':
        SERVO_STEP_RATE1 = constrain(SERVO_STEP_RATE1 + 5, SERVO_MIN, SERVO_MAX);
        servo1.write(SERVO_STEP_RATE1);
        break;

      // Servo2 controls
      case 'Q':
        SERVO_STEP_RATE2 = constrain(SERVO_STEP_RATE2 + 15, SERVO_MIN, SERVO_MAX);
        servo2.write(SERVO_STEP_RATE2);
        break;

      case 'E':
        SERVO_STEP_RATE2 = constrain(SERVO_STEP_RATE2 - 15, SERVO_MIN, SERVO_MAX);
        servo2.write(SERVO_STEP_RATE2);
        break;

      // Buzzer beep
      case 'M':
        tone(buzz, 2000);
        delay(500);
        noTone(buzz);
        break;
    }
  }
}


/* =====================================================
   ================= MAIN LOOP =========================
   ===================================================== */

/*
Function Name : loop
Input         : None
Output        : None
Logic         :
  - Reads Bluetooth input every cycle
  - Runs angle loop 7 times for each velocity loop
  - Ensures angle loop runs faster than velocity loop
Example Call  : Automatically called repeatedly by Arduino
*/
unsigned int counter = 0;

void loop() {

  // Read commands from Serial/Bluetooth
  handleBluetoothInput();

  // Run inner loop multiple times (fast stabilization)
  if (counter < 7) {
    counter++;
    runAngleLoop();
  }

  // Run outer loop once every 7 cycles (slower velocity control)
  if (counter == 7) {
    counter = 0;
    runVelocityLoop();
  }
}


/* =====================================================
   ============ OUTER LOOP : VELOCITY PID ===============
   ===================================================== */

/*
Function Name : runVelocityLoop
Input         : None
Output        : Updates targetAngle_cmd (desired tilt angle)
Logic         :
  - Computes dt using micros()
  - Reads encoder count safely
  - Computes wheel velocity (pulses/sec)
  - Runs velocity PID to generate tilt angle command
Example Call  : runVelocityLoop();
*/
void runVelocityLoop() {

  // Compute time step for velocity loop
  currentTime = micros();
  dt = (currentTime - previousTime) * 1e-6;
  previousTime = currentTime;

  // Safety check
  if (dt <= 0) return;

  // Read encoder count safely (interrupt may modify it)
  noInterrupts();
  long right = wheel_pulse_count_right;
  interrupts();

  // Compute encoder delta pulses
  long delta_right = right - prev_right;
  prev_right = right;

  // Compute velocity (pulses/sec)
  // Negative sign used because encoder direction is inverted in your setup
  currentVelocity = -delta_right / dt;

  // Velocity error = desired - measured
  error_Vel = targetVelocity - currentVelocity;

  // Integral term accumulation
  integral_Vel += error_Vel * dt;
  integral_Vel = constrain(integral_Vel, -3000, 3000);

  // Derivative of velocity error
  double dVel = (error_Vel - prevError_Vel) / dt;

  // Low-pass filter derivative (reduces noise)
  filteredD_Pos = (1 - alpha1) * filteredD_Pos + alpha1 * dVel;

  // Save error for next derivative computation
  prevError_Vel = error_Vel;

  // PID output becomes desired tilt angle command
  targetAngle_cmd =
      (Kp_Vel * error_Vel) +
      (ki_Vel * integral_Vel) +
      (kd_Vel * filteredD_Pos);

  // Limit tilt command to avoid falling
  targetAngle_cmd = constrain(targetAngle_cmd, -8, 8);
}


/* =====================================================
   ============== INNER LOOP : ANGLE PID ================
   ===================================================== */

/*
Function Name : runAngleLoop
Input         : None
Output        : Motor PWM signals (via controlMotors)
Logic         :
  - Reads MPU angle
  - Runs balancing PID (angle control)
  - Measures yaw rate and runs yaw PID
  - Mixes yaw correction into left/right motor PWM
  - Sends final PWM to motors
Example Call  : runAngleLoop();
*/
void runAngleLoop() {

  // Compute dt for inner loop
  currentTime1 = micros();
  dt1 = (currentTime1 - previousTime1) * 1e-6;
  previousTime1 = currentTime1;

  // Update MPU6050 internal filters
  mpu.update();

  // Read tilt angle (Y axis tilt) and apply mounting offset correction
  currentAngle = mpu.getAngleY() - AngleOffset;

  /* ================= ANGLE PID ================= */

  // Error = target angle - measured angle
  error_Ang = targetAngle_cmd - currentAngle;

  // Integral term
  integral_Ang += error_Ang * dt1;
  integral_Ang = constrain(integral_Ang, -40, 40);

  // Derivative term
  double dAng = (error_Ang - prevError_Ang) / dt1;

  // Filter derivative to reduce noise
  filteredD_Ang = (1 - alpha2) * filteredD_Ang + alpha2 * dAng;

  // Store error for next loop
  prevError_Ang = error_Ang;

  // Compute motor command from PID
  double motorCommand =
      (Kp * error_Ang) +
      (Ki * integral_Ang) +
      (Kd * filteredD_Ang);

  // Apply friction compensation
  if (motorCommand > 0) motorCommand += frictionOffset;
  else if (motorCommand < 0) motorCommand -= frictionOffset;

  // Clamp command to valid PWM range
  motorCommand = constrain(motorCommand, -255, 255);


  /* ================= YAW RATE MEASUREMENT ================= */

  // Gyro Z axis gives yaw angular velocity (deg/sec)
  yawRateMeasured = mpu.getGyroZ();


  /* ================= YAW PID ================= */

  // Yaw error = target yaw rate - measured yaw rate
  error_Yaw = targetYaw_RATE - yawRateMeasured;

  // Integral term for yaw
  integral_Yaw += error_Yaw * dt1;
  integral_Yaw = constrain(integral_Yaw, -50, 50);

  // Derivative term
  double dYaw = (error_Yaw - prevError_Yaw) / dt1;

  // Filter yaw derivative
  filteredD_Yaw = (1 - alphaYaw) * filteredD_Yaw + alphaYaw * dYaw;

  // Store yaw error
  prevError_Yaw = error_Yaw;

  // Compute yaw correction command
  yawCommand =
      (Kp_Yaw * error_Yaw) +
      (Ki_Yaw * integral_Yaw) +
      (Kd_Yaw * filteredD_Yaw);

  // Limit yaw correction so it doesn't overpower balancing
  yawCommand = constrain(yawCommand, -80, 80);


  /* ================= MOTOR MIXING ================= */

  // Proper yaw mixing:
  // To turn right: left motor faster, right motor slower
  double leftCmd  = motorCommand - yawCommand;
  double rightCmd = motorCommand + yawCommand;

  // Convert signed motor signals into absolute PWM magnitudes
  leftCmd  = constrain(abs(leftCmd),  0, 255);
  rightCmd = constrain(abs(rightCmd), 0, 255);

  // Send commands to motor driver
  controlMotors(motorCommand, leftCmd, rightCmd);
}


/* =====================================================
   ================= MOTOR CONTROL =====================
   ===================================================== */

/*
Function Name : controlMotors
Input         :
  cmd        : Signed motor command (decides direction)
  leftSpeed  : PWM value for left motor (0-255)
  rightSpeed : PWM value for right motor (0-255)
Output        : None
Logic         :
  - If cmd > 0 → forward direction
  - If cmd < 0 → reverse direction
  - Applies PWM speeds to enable pins
Example Call  : controlMotors(motorCommand, leftPWM, rightPWM);
*/
void controlMotors(double cmd, int leftSpeed, int rightSpeed) {

  // Direction logic based on sign of cmd
  if (cmd > 0) {

    // Forward direction
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);

    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);

  } else {

    // Reverse direction
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);

    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, HIGH);
  }

  // Apply PWM speed
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}


/* =====================================================
   ================= ENCODER ISR =======================
   ===================================================== */

/*
Function Name : ISR_rightA
Input         : Hardware interrupt triggered on encodPinAR change
Output        : Updates wheel_pulse_count_right
Logic         :
  - Quadrature decoding:
    If A == B, direction is forward → count++
    Else direction reverse → count--
Example Call  : Automatically called by interrupt
*/
void ISR_rightA() {

  // Quadrature direction detection
  if (digitalRead(encodPinAR) == digitalRead(encodPinBR))
    wheel_pulse_count_right++;
  else
    wheel_pulse_count_right--;
}
