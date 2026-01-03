//#define PRINT_DEBUG_BUILD   // Enable Serial debug prints when required

/* ==============================================================
                          LIBRARIES
   ============================================================== */

/*
 Library: PID_v1
 Purpose : Implements discrete PID control algorithm
*/
#include <PID_v1.h>

/*
 Library: Servo
 Purpose : Controls standard RC servo motors
*/
#include <Servo.h>

/*
 Library: I2Cdev
 Purpose : Low-level I2C communication abstraction
*/
#include "I2Cdev.h"

/*
 Library: MPU6050 DMP Driver
 Purpose : Provides access to MPU6050 sensor and DMP features
*/
#include "MPU6050_6Axis_MotionApps20.h"

/*
 Conditional inclusion of Arduino Wire library
*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

/* ==============================================================
                          MPU6050 VARIABLES
   ============================================================== */

/*
 Variable Name: mpu
 Description  : MPU6050 object handling IMU sensor and DMP
 Range        : Hardware interface object
*/
MPU6050 mpu;

/*
 Variable Name: dmpReady
 Description  : Indicates whether DMP initialization was successful
 Range        : true / false
*/
bool dmpReady = false;

/*
 Variable Name: devStatus
 Description  : Status code returned by DMP initialization
                0 = success, non-zero = error
 Range        : 0–255
*/
uint8_t devStatus;

/*
 Variable Name: packetSize
 Description  : Size (in bytes) of one DMP FIFO packet
 Range        : Typically 42 or 48 bytes
*/
uint16_t packetSize;

/*
 Variable Name: fifoBuffer
 Description  : Buffer storing raw DMP FIFO data
 Range        : Byte array [0–63]
*/
uint8_t fifoBuffer[64];

/*
 Variable Name: q
 Description  : Quaternion representing orientation
*/
Quaternion q;

/*
 Variable Name: gravity
 Description  : Gravity vector extracted from quaternion
*/
VectorFloat gravity;

/*
 Variable Name: gy
 Description  : Raw gyroscope readings (X, Y, Z axes)
*/
VectorInt16 gy;

/*
 Variable Name: ypr
 Description  : Yaw, Pitch, Roll angles from DMP
 Range        : Radians
 Index        : [0] Yaw, [1] Pitch, [2] Roll
*/
float ypr[3];

/* ==============================================================
                          PID PARAMETERS
   ============================================================== */

/*
 Variable Name: PID_MIN_LIMIT / PID_MAX_LIMIT
 Description  : Output saturation limits for motor PWM
 Range        : -255 to +255
*/
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT  255

/*
 Variable Name: PID_SAMPLE_TIME
 Description  : PID computation interval in milliseconds
 Range        : 5–20 ms recommended
*/
#define PID_SAMPLE_TIME 10

/*
 Variable Name: SETPOINT_PITCH_ANGLE_OFFSET
 Description  : Static balance angle offset due to COM shift
 Range        : Approximately -15° to +15°
*/
#define SETPOINT_PITCH_ANGLE_OFFSET -8

/*
 Variable Name: MIN_ABSOLUTE_SPEED
 Description  : Minimum PWM applied to overcome motor dead zone
 Range        : 0–50
*/
#define MIN_ABSOLUTE_SPEED 15

/*
 Variable Name: setpointPitchAngle
 Description  : Desired pitch angle for self-balancing
 Range        : Degrees
*/
double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;

/*
 Variable Name: pitchGyroAngle
 Description  : Measured pitch angle from MPU DMP
 Range        : Degrees
*/
double pitchGyroAngle = 0;

/*
 Variable Name: pitchPIDOutput
 Description  : Output of pitch PID controller
 Range        : -255 to +255
*/
double pitchPIDOutput = 0;

/*
 Variable Name: setpointYawRate
 Description  : Desired yaw rate (no rotation by default)
 Range        : Degrees/second
*/
double setpointYawRate = 0;

/*
 Variable Name: yawGyroRate
 Description  : Measured yaw angular velocity
 Range        : Raw gyro units
*/
double yawGyroRate = 0;

/*
 Variable Name: yawPIDOutput
 Description  : Yaw correction term added to motors
 Range        : -60 to +60
*/
double yawPIDOutput = 0;

/* ==============================================================
                          PID OBJECTS
   ============================================================== */

/*
 Pitch PID Controller
 Purpose : Maintains upright balance (inner loop)
*/
PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle,
             28.0, 0.8, 1.2, DIRECT);

/*
 Yaw PID Controller
 Purpose : Provides heading stabilization
*/
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate,
           1.0, 0.3, 0.0, DIRECT);

/* ==============================================================
                          MOTOR PINS
   ============================================================== */

/*
 Left Motor (L298N Channel A)
*/
#define EN1 6
#define IN1 A2
#define IN2 A3

/*
 Right Motor (L298N Channel B)
*/
#define EN2 5
#define IN3 9
#define IN4 4

/* ==============================================================
                          ENCODERS
   ============================================================== */

/*
 Variable Name: ENC_L_A / ENC_L_B
 Description  : Left motor encoder channels
*/
#define ENC_L_A 7
#define ENC_L_B 8

/*
 Variable Name: ENC_R_A / ENC_R_B
 Description  : Right motor encoder channels
*/
#define ENC_R_A 2
#define ENC_R_B 3

/*
 Variable Name: encoderLeft / encoderRight
 Description  : Encoder pulse counters
 Range        : 0 to large integer (depends on runtime)
*/
volatile long encoderLeft = 0;
volatile long encoderRight = 0;

/* ==============================================================
                          OTHER DEVICES
   ============================================================== */

/*
 Variable Name: BUZZER_PIN
 Description  : Buzzer control pin
*/
#define BUZZER_PIN A1

/*
 Variable Name: SERVO1_PIN / SERVO2_PIN
 Description  : Servo control pins
*/
#define SERVO1_PIN 11
#define SERVO2_PIN 10

Servo servo1;
Servo servo2;

/* ==============================================================
 Function Name : encoderLeftISR
 Input         : None
 Output        : None
 Logic         : Increments left encoder count on rising edge
 Example Call  : Triggered by hardware interrupt
 ============================================================== */
void encoderLeftISR() {
  encoderLeft++;
}

/* ==============================================================
 Function Name : encoderRightISR
 Input         : None
 Output        : None
 Logic         : Increments right encoder count on rising edge
 Example Call  : Triggered by hardware interrupt
 ============================================================== */
void encoderRightISR() {
  encoderRight++;
}

/* ==============================================================
 Function Name : rotateMotor
 Input         :
   speedL → Left motor command (-255 to +255)
   speedR → Right motor command (-255 to +255)
 Output        : None
 Logic         :
   • Determines motor direction
   • Adds dead-zone compensation
   • Applies PWM to L298N driver
 Example Call  : rotateMotor(120, -80);
 ============================================================== */
void rotateMotor(int speedL, int speedR) {

  // Left Motor Direction
  if (speedL < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Right Motor Direction (inverted wiring)
  if (speedR < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  speedL = constrain(abs(speedL) + MIN_ABSOLUTE_SPEED, 0, 255);
  speedR = constrain(abs(speedR) + MIN_ABSOLUTE_SPEED, 0, 255);

  analogWrite(EN1, speedL);
  analogWrite(EN2, speedR);
}

/* ==============================================================
 Function Name : setup
 Input         : None
 Output        : None
 Logic         :
   • Initializes I2C, motors, encoders, MPU6050, PID controllers
   • Plays startup buzzer
 Example Call  : Automatically executed at boot
 ============================================================== */
void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

#ifdef PRINT_DEBUG_BUILD
  Serial.begin(115200);
#endif

  // Motor pin setup
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pin setup
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightISR, RISING);

  // Buzzer startup indication
  pinMode(BUZZER_PIN, OUTPUT);
  tone(BUZZER_PIN, 2000, 150);
  delay(200);
  noTone(BUZZER_PIN);

  // Servo initialization
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(90);
  servo2.write(90);

  // MPU6050 initialization
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-4092);
  mpu.setYAccelOffset(687);
  mpu.setZAccelOffset(1430);
  mpu.setXGyroOffset(132);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(47);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  }

  // PID configuration
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME);
  pitchPID.SetMode(AUTOMATIC);

  yawPID.SetOutputLimits(-60, 60);
  yawPID.SetSampleTime(PID_SAMPLE_TIME);
  yawPID.SetMode(AUTOMATIC);

  rotateMotor(0, 0);
}

/* ==============================================================
 Function Name : loop
 Input         : None
 Output        : None
 Logic         :
   • Reads IMU data
   • Computes PID outputs
   • Drives motors for balance
 Example Call  : Runs continuously
 ============================================================== */
void loop() {

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    pitchGyroAngle = ypr[1] * 180.0 / M_PI;
    yawGyroRate = gy.z;

    pitchPID.Compute();
    yawPID.Compute();

    int leftMotor  = pitchPIDOutput + yawPIDOutput;
    int rightMotor = pitchPIDOutput - yawPIDOutput;

    rotateMotor(leftMotor, rightMotor);

#ifdef PRINT_DEBUG_BUILD
    Serial.print("Pitch: ");
    Serial.print(pitchGyroAngle);
    Serial.print(" | EncL: ");
    Serial.print(encoderLeft);
    Serial.print(" | EncR: ");
    Serial.println(encoderRight);
#endif
  }
}
