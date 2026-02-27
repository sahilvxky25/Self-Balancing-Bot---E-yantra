/*
  Team Id:          KB_1321
  Author List:      Sahil Kumar Patra, Somya Ranjan Suar,
                    Aiyush Anand, H Noah Siddhant
  Filename:         eYRC_KB_1321_Task6_Documented_Code.ino
  Theme:            eYRC Krishi Balancer 2025-26
  Functions:        setup, loop, handleBluetoothInput,
                    runVelocityLoop, runAngleLoop,
                    controlMotors, ISR_rightA
  Global Variables: mpu, BT_RX,BT_TX servo1, servo2,
                    motor1A, motor1B, enable1,
                    motor2A, motor2B, enable2,
                    buzz,
                    Kp_Ang, Ki_Ang, Kd_Ang,
                    Kp_Vel, ki_Vel, kd_Vel,
                    Kp_Yaw, Ki_Yaw, Kd_Yaw,
                    alpha_Vel, alpha_Ang,
                    frictionOffset, AngleOffset,
                    targetVelocity, targetYaw_RATE,
                    GYRO_X_OFFSET,GYRO_Y_OFFSET,GYRO_Z_OFFSET,
                    ACC_X_OFFSET,ACC_Y_OFFSET,ACC_Z_OFFSET
                    currentAngle, currentVelocity,
                    prev_right, prev_left,
                    wheel_pulse_count_right,
                    error_Ang, prevError_Ang,
                    integral_Ang, filteredD_Ang,
                    filteredD_Vel,
                    targetAngle_cmd,
                    error_Vel, prevError_Vel,
                    integral_Vel,
                    error_Yaw, prevError_Yaw,
                    integral_Yaw, filteredD_Yaw,
                    yawRateMeasured, yawCommand,
                    alphaYaw,
                    VEL_STEP,YAW_STEP_RATE,
                    currentTime, previousTime,
                    currentTime1, previousTime1,
                    dt, dt1,
                    SERVO_STEP_RATE1,
                    SERVO_STEP_RATE2,
                    counter
 */
#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/* =====================================================
   ================= GLOBAL OBJECTS ====================
   ===================================================== */

/*
Variable Name: mpu
Description : MPU6050 IMU object used for angle estimation
Range       : Internal to library
*/
MPU6050 mpu(Wire);
/* =====================================================
   ================= MOTOR PINS ========================
   ===================================================== */

/*
Variable Name: motor1A, motor1B
Description : Direction control pins for left motor
Range       : Digital HIGH / LOW
*/
int motor1A = A3, motor1B = A2;

/*
Variable Name: enable1
Description : PWM pin for left motor speed control
Range       : 0 – 255
*/
int enable1 = 6;

/*
Variable Name: motor2A, motor2B
Description : Direction control pins for right motor
Range       : Digital HIGH / LOW
*/
int motor2A = 9, motor2B = 4;

/*
Variable Name: enable2
Description : PWM pin for right motor speed control
Range       : 0 – 255
*/

int enable2 = 5;

/*
Variable Name: buzz
Description : Buzzer pin for audio feedback
Range       : Digital output
*/
const int buzz = A1;

/* =====================================================
   ================= PID GAINS =========================
   ===================================================== */

/*
Inner Loop PID (Angle Control)
Purpose: Maintains upright balance
*/
double Kp_Ang = 30;     // Proportional gain
double Ki_Ang = 200;    // Integral gain
double Kd_Ang = 1;   // Derivative gain
/*
Variable Name: prev_right, prev_left
Description : Previous encoder counts used for velocity calculation
Range       : Signed long integer
*/
long prev_right=0;
long prev_left=0;
/*
Outer Loop PID (Velocity Control)
Purpose: Converts velocity error into target angle
*/

double Kp_Vel = 0.00105; // Proportional gain
double ki_Vel = 0.000; // Integral gain
double kd_Vel = 0.000047; // Derivative gain
/*
Variable Names : alpha_Vel,alpha_Ang,alphaYaw
Purpose : Control the degree of smootihing or attenuation in the filter's Output
Range : (0-1)
*/
double alpha_Vel = 0.7;
double alpha_Ang = 0.7;
double alphaYaw = 0.7;  
/* =====================================================
   ================= SYSTEM PARAMETERS =================
   ===================================================== */

/*
Variable Name: frictionOffset
Description : Compensates static friction in motors
Range       : Typically 10 – 30
*/
int frictionOffset = 8;

/*
Variable Name: AngleOffset
Description : IMU mounting offset correction
Range       : Small value in degrees
*/
double AngleOffset = 1.07;

/* =====================================================
   ================= TARGET VARIABLES ==================
   ===================================================== */

/*
Variable Name: targetVelocity
Description : Desired forward/backward velocity
Range       : Encoder counts per loop
*/
double targetVelocity = 0;

/*
Variable Name: targetYaw_RATE
Description : Desired turning rate
Range       : ± few PWM units
*/
double targetYaw_RATE = 0;

/* =====================================================
   ================= MPU OFFSETS =======================
   ===================================================== */

/*
Gyroscope and accelerometer calibration offsets
Obtained from MPU Calibration Code
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
Variable Name: currentAngle
Description : Current robot tilt angle
Range       : Degrees
*/
double currentAngle = 0;

/*
Variable Name: currentVelocity
Description : Measured velocity from encoders
Range       : Encoder counts
*/
double currentVelocity = 0;

/* -------- Angle PID States -------- */
/*
Variable Names : error_Ang, prevError_Ang,integral_Ang
Description : Used in the Calculation of Motor Command
Range : Mpu Readings
*/
double error_Ang = 0;
double prevError_Ang = 0;
double integral_Ang = 0;

/*
Variable Names : filteredD_Ang
Description : Give the Out after Applying Low pass Filter
Range : Value received after calculating Derivative of Angle Error from Mpu Reading
*/

double filteredD_Ang = 0;
/*
Variable Names : filteredD_Vel
Description : Give the Out after Applying Low pass Filter
Range : Value received after calculating Derivative of Vel Error from Encoder Reading
*/
double filteredD_Vel = 0;
/*
Variable Name: targetAngle_cmd
Description : Angle command generated by velocity loop
Range       : Small angle in degrees
*/
volatile double targetAngle_cmd = 0.0;

/* -------- Velocity PID States -------- */
/*
Variable Names : error_Vel, prevError_Vel,integral_Vel
Description : Used in the Calculation of yaw
Range : Encoder Readings
*/
double error_Vel = 0;
double prevError_Vel = 0;
double integral_Vel = 0;
/* =====================================================
   ================= YAW PID (RATE LOOP) ===============
   ===================================================== */
/*
Yaw Loop PID (Yaw Control)
Purpose: Converts Yaw error into motor Command
*/

double Kp_Yaw = 0.7;
double Ki_Yaw = 0;
double Kd_Yaw = 0;
/*
Variable Names : error_Yaw, prevError_Yaw,integral_Yaw
Description : Used in the Calculation of yawCommand
Range : Gyro Measurements
*/
double error_Yaw = 0;
double prevError_Yaw = 0;
double integral_Yaw = 0;
double filteredD_Yaw = 0;

double yawRateMeasured = 0;   // deg/sec
double yawCommand = 0;        // correction added to motors


/* =====================================================
   ================= CONTROL LIMITS ====================
   ===================================================== */
/*
Variable Name: VEL_STEP
Description : Increment step applied to targetVelocity on 'F' command
Range       : Positive double (experimentally tuned)
*/
const double VEL_STEP = 320;

/*
Variable Name: YAW_STEP_RATE
Description : Turning rate step applied on 'L'/'R' commands
Range       : Positive double (deg/sec equivalent control unit)
*/
const double YAW_STEP_RATE = 35;


/* =====================================================
   ================= TIME VARIABLES ====================
   ===================================================== */

/*
Variable Name: dt
Description : Loop time difference
Range       : Seconds
*/
unsigned long currentTime, previousTime = 0;
double dt;
unsigned long currentTime1,previousTime1 = 0;
double dt1;
/* =====================================================
   ================= BLUETOOTH =========================
   ===================================================== */
//Defining the pins of HC-05
#define BT_RX 0
#define BT_TX 1
SoftwareSerial BT(BT_RX, BT_TX);

/* =====================================================
   ================= ENCODERS ==========================
   ===================================================== */
//Defining the pins 0f motor encoders
#define encodPinAR 2
#define encodPinBR 3
#define encodPinAL 7
#define encodPinBL 8
/*
Variable Name: wheel_pulse_count_right
Description : Right Wheel Encoder Count
Range       : Encoder Count
*/
volatile long wheel_pulse_count_right = 0;

/* =====================================================
   ================= SERVOS ============================
   ===================================================== */
//Defining the pins of servo motors
#define SERVO1_PIN 10
#define SERVO2_PIN 11

Servo servo1, servo2;
/*
Variable Name: SERVO_STEP_RATE1, SERVO_STEP_RATE2
Description : Define the Angle to which the servo has to reach
Range       : (0-180) Degree
*/
double SERVO_STEP_RATE1 = 0;
double SERVO_STEP_RATE2 = 0;
/*
Variable Name: SERVO_MIN, SERVO_MAX
Description : Define the Minimum and Maximum Angle to which the servo can reach
Range       : (0-180) Degree
*/
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

/* =====================================================
   ====================== SETUP ========================
   ===================================================== */

/*
Function Name : setup
Input         : None
Output        : None
Logic         :Performs system hardware initialization.
                - Initializes Serial communication and I2C bus
                - Initializes MPU6050 and applies calibration offsets
                - Configures motor control pins as OUTPUT
                - Configures encoder input pins with pull-ups
                - Attaches interrupt for right encoder channel A
                - Attaches servo motors to respective PWM pins
                - Sets initial servo positions
Example Call  : Automatically called by Arduino
*/
void setup() {
  Serial.begin(9600);
// Wire.begin() is a built-in Arduino function from the Wire library for I2C communication
  Wire.begin();
/*
mpu.begin() is a method from the MPU6050_light Arduino library that initializes the sensor,
 wakes it from sleep mode, 
 sets default or specified scales for accelerometer and gyroscope, 
 and verifies communication.
*/
  mpu.begin();
/*
mpu.setGyroOffsets(x, y, z) → Subtracts these values from raw gyroscope readings (deg/s).
mpu.setAccOffsets(x, y, z) → Subtracts these values from raw accelerometer readings (in g or raw units depending on range).
*/
  mpu.setGyroOffsets(GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
  mpu.setAccOffsets(ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET);

  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(enable2, OUTPUT);

  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
/*
attachInterrupt(digitalPinToInterrupt(encodPinAR), ISR_rightA, CHANGE) 
  It is an Arduino function call that attaches an interrupt service routine (ISR) to a specific pin, 
  triggering the ISR on signal changes—commonly used for fast event detection like encoder pulses
*/
  attachInterrupt(digitalPinToInterrupt(encodPinAR), ISR_rightA, CHANGE);
/*
servo.attach() is a method from the Arduino Servo library that attaches (initializes) a Servo object to a specific PWM-capable pin, 
enabling the Arduino to control the servo motor's position 
*/
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
/*
servo.write() is a method from the Arduino Servo library that sends a position or
 speed command to the attached servo motor,
 controlling its shaft.
*/
  servo1.write(SERVO_STEP_RATE1);
  servo2.write(SERVO_STEP_RATE2);
}


/*
Function Name : handleBluetoothInput()
Input         : None (reads Serial buffer)
Output        : None
Logic         :Interprets incoming Bluetooth/Serial commands to update:
              - AngleOffset calibration
              - targetVelocity (forward/backward motion)
              - targetYaw_RATE (turning rate)
              - Servo positions (SERVO_STEP_RATE1/2)
              - Buzzer activation
Example Call  : handleBluetoothInput();
*/
void handleBluetoothInput() {
  /*
  Function Name : Serial.available()
  Input : None
  Output: None
  Logic : It a built-in Arduino function that returns the number of bytes (characters) currently available for reading from the serial receive buffer.
  */
  while (Serial.available()) {
    /*
    Function Name : Serial.available()
    Input : None
    Output: The next byte as an int (0–255) if data is available
            - (-1) if the buffer is empty (nothing to read)
    Logic : 
        It a built-in Arduino function that reads and removes the next available byte (character) from the serial receive buffer. 
    */
    char cmd = Serial.read();
    switch (cmd) {
      case '+': AngleOffset += 0.02; break;
      case '-': AngleOffset -= 0.02; break;
      case 'F':
        targetVelocity+=VEL_STEP;
        break;
      case 'B':
        targetVelocity-=VEL_STEP*2;
        break;
      case 'R': targetYaw_RATE =  YAW_STEP_RATE; break;
      case 'L': targetYaw_RATE = -YAW_STEP_RATE; break;
      case 'S': targetYaw_RATE = 0; targetVelocity = 0; break;
      case 'U': servo1.write(22); 
                servo2.write(158); 
                break;
      case 'D': servo2.write(0);break;
      case 'Q': servo1.write(0);break;
      case 'E': break;
      case 'M': tone(buzz, 2000, 1500);break;
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
Logic         :Main scheduler controlling execution frequency of loops.
              - Continuously checks Bluetooth input
              - Executes runAngleLoop() at higher frequency
              - Executes runVelocityLoop() periodically using counter
*/
unsigned int counter = 0;

void loop() {
  
  handleBluetoothInput();

  if (counter < 7) {
    counter++;
    runAngleLoop();
  }

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
Output        : Updates targetAngle_cmd
Logic         :Implements outer PID loop for velocity control.
                  - Computes elapsed time (dt)
                  - Reads encoder count safely
                  - Calculates wheel velocity from pulse difference
                  - Computes velocity error
                  - Updates integral and derivative terms
                  - Applies PID gains (Kp_Vel, ki_Vel, kd_Vel)
                  - Generates target tilt angle command
                  - Constrains output to safe range
Example Call  : runvelocityLoop();
*/
void runVelocityLoop() {

  currentTime = micros();
  dt = (currentTime - previousTime) * 1e-6;
  previousTime = currentTime;

  if (dt <= 0) return;

  // Disable interrupts to safely read encoder count
noInterrupts();
long right = wheel_pulse_count_right;
interrupts();

// Calculate change in encoder count since last loop
long delta_right = right - prev_right;

// Store current count for next iteration
prev_right = right;

// Compute actual wheel velocity (counts/sec)
currentVelocity = -delta_right/dt;

// Velocity error between desired and actual velocity
error_Vel = targetVelocity - currentVelocity;

// Integral term accumulation for velocity PID
integral_Vel += error_Vel * dt;

// Limit integral to prevent windup
integral_Vel = constrain(integral_Vel, -3000, 3000);

// Derivative of velocity error
double dVel = (error_Vel - prevError_Vel) / dt;

// Apply low pass filter to derivative term
filteredD_Vel = (1-alpha_Vel)*filteredD_Vel + alpha_Vel*dVel;

// Store error for next loop
prevError_Vel = error_Vel;

// Convert velocity PID output to angle command
targetAngle_cmd =
    (Kp_Vel * error_Vel) +
    (ki_Vel * integral_Vel) +
    (kd_Vel * filteredD_Vel);

// Limit commanded tilt angle for safety
targetAngle_cmd = constrain(targetAngle_cmd, -12, 12);
}
/* =====================================================
   ============== INNER LOOP : ANGLE PID ================
   ===================================================== */

/*
Function Name : runAngleLoop
Input         : None

Output        : Motor PWM signals
Logic         : Implements inner PID loop for balance control.
                  - Updates MPU6050 readings
                  - Calculates current tilt angle
                  - Computes angle error
                  - Updates integral and filtered derivative terms
                  - Applies PID gains (Kp_Ang, Ki_Ang, Kd_Ang)
                  - Adds friction compensation
                  - Limits motor command range
                  - Measures yaw rate from gyro
                  - Executes yaw rate PID
                  - Mixes balance and yaw commands
                  - Generates left and right PWM values
Example Call  : runAngleLoop();
*/
void runAngleLoop() {
  currentTime1 = micros();
  dt1 = (currentTime1 - previousTime1) * 1e-6;
  previousTime1 = currentTime1;

  mpu.update();
//====Angle PID=======
// Get tilt angle from MPU6050 and compensate mounting offset
currentAngle = mpu.getAngleY() - AngleOffset;

// Compute angle error for balancing
error_Ang = targetAngle_cmd - currentAngle;

// Integral accumulation for angle PID
integral_Ang += error_Ang * dt1;

// Limit integral to prevent instability
integral_Ang = constrain(integral_Ang, -40, 40);

// Compute derivative of angle error
double dAng = (error_Ang - prevError_Ang) / dt1;

// Apply low pass filter to derivative term
filteredD_Ang = (1 - alpha_Ang) * filteredD_Ang + alpha_Ang * dAng;

// Store previous error for next iteration
prevError_Ang = error_Ang;

// PID output to control motor torque
double motorCommand =
    (Kp_Ang * error_Ang) +
    (Ki_Ang * integral_Ang) +
    (Kd_Ang * filteredD_Ang);

// Add friction compensation to overcome motor dead zone
if (motorCommand > 0) motorCommand += frictionOffset;
else if (motorCommand < 0) motorCommand -= frictionOffset;

// Limit PWM command to valid range
motorCommand = constrain(motorCommand, -255, 255);

// ================= YAW RATE MEASUREMENT =================
 // Get yaw angular velocity from gyro (deg/sec)
yawRateMeasured = mpu.getGyroZ();

// Compute yaw rate error
error_Yaw = targetYaw_RATE - yawRateMeasured;

// Integral accumulation for yaw PID
integral_Yaw += error_Yaw * dt1;

// Prevent integral windup
integral_Yaw = constrain(integral_Yaw, -50, 50);

// Derivative of yaw error
double dYaw = (error_Yaw - prevError_Yaw) / dt1;

// Low pass filter for yaw derivative
filteredD_Yaw = (1 - alphaYaw) * filteredD_Yaw + alphaYaw * dYaw;

// Store previous yaw error
prevError_Yaw = error_Yaw;

  yawCommand =
      (Kp_Yaw * error_Yaw) +
      (Ki_Yaw * integral_Yaw) +
      (Kd_Yaw * filteredD_Yaw);

  yawCommand = constrain(yawCommand, -80, 80);

// ================= PROPER MIXING =================
// Combine balance and yaw commands for left & right motors
double leftCmd  = motorCommand - yawCommand;
double rightCmd = motorCommand + yawCommand;

// Convert to absolute PWM values
leftCmd  = constrain(abs(leftCmd),  0, 255);
rightCmd = constrain(abs(rightCmd), 0, 255);

  controlMotors(motorCommand,leftCmd, rightCmd);
}


/* =====================================================
   ================= MOTOR CONTROL =====================
   ===================================================== */

/*
Function Name : controlMotors
Input         :
  - cmd        : Sign determines direction
  - leftSpeed  : PWM for left motor
  - rightSpeed : PWM for right motor
Output        : None
Logic         :Drives motor driver module.
                - Sets motor direction based on sign of cmd
                - Applies PWM signals to enable pins
*/
void controlMotors(double cmd, int leftSpeed, int rightSpeed) {
  // Determine motor rotation direction based on command sign
  if (cmd > 0) {
    digitalWrite(motor1A, HIGH); digitalWrite(motor1B, LOW);
    digitalWrite(motor2A, HIGH); digitalWrite(motor2B, LOW);
  } else {
    digitalWrite(motor1A, LOW); digitalWrite(motor1B, HIGH);
    digitalWrite(motor2A, LOW); digitalWrite(motor2B, HIGH);
  }
  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}

/* =====================================================
   ================= ENCODER ISR =======================
   ===================================================== */

/*
Function Name : ISR_rightA
Input         : Hardware interrupt
Output        : Updates wheel_pulse_count_right
Logic         : Triggered on both rising and falling edges (CHANGE mode) to maximize resolution.
                  Only right encoder is used due to Nano interrupt limitations.
                  - Quadrature encoder interrupt routine.
                  - Reads encoder A and B signals
                  - Determines direction of wheel rotation
                  - Increments or decrements pulse counter
Example Call  : ISR_rightA();
*/
void ISR_rightA() {
  // Determine direction of wheel rotation using quadrature encoding
  if (digitalRead(encodPinAR) == digitalRead(encodPinBR))
    wheel_pulse_count_right++;
  else
    wheel_pulse_count_right--;
}

/*
Additional Comment :
    Currently, only the right wheel encoder is being utilized, as it is connected to interrupt-enabled pins on the Arduino Nano. 
    The left wheel encoder is connected to regular digital pins that do not support external interrupts.
    The current code is written under the assumption that the left encoder count is equal to the right encoder count.
    Basis of the assumption:
        During hardware validation tests, we observed that equal amounts of physical wheel rotation produce the same Δ (change) in encoder counts for both the left and right encoders.
*/