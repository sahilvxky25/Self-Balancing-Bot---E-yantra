//#define PRINT_DEBUG_BUILD   // Uncomment to enable Serial debug prints

#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* ========================= I2C CONFIG =========================
   Uses Arduino Wire implementation for I2C communication
*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

/* ==============================================================
                          MPU6050 SECTION
   ============================================================== */

/*
 Variable Name: mpu
 Description  : MPU6050 IMU object handling accelerometer + gyroscope + DMP
 Range        : N/A (hardware interface)
*/
MPU6050 mpu;

/*
 Variable Name: INTERRUPT_PIN
 Description  : Arduino pin connected to MPU6050 INT pin
 Range        : Digital pin number
*/
#define INTERRUPT_PIN 2   // D2 as per PCB design

/*
 Variable Name: dmpReady
 Description  : Flag indicating whether DMP initialization succeeded
 Range        : true / false
*/
bool dmpReady = false;

/*
 Variable Name: mpuIntStatus
 Description  : Holds MPU interrupt status register value
 Range        : 0–255
*/
uint8_t mpuIntStatus;

/*
 Variable Name: devStatus
 Description  : Status returned by DMP initialization
                0 → success, non-zero → failure
 Range        : 0–255
*/
uint8_t devStatus;

/*
 Variable Name: packetSize
 Description  : Size (in bytes) of one DMP FIFO packet
 Range        : typically 42 or 48 bytes
*/
uint16_t packetSize;

/*
 Variable Name: fifoBuffer
 Description  : Buffer used to store FIFO data from MPU6050
 Range        : byte array
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
 Description  : Raw gyroscope readings (X, Y, Z)
*/
VectorInt16 gy;

/*
 Variable Name: ypr
 Description  : Yaw, Pitch, Roll angles in radians
 Index        : [0]=Yaw, [1]=Pitch, [2]=Roll
*/
float ypr[3];

/*
 Variable Name: mpuInterrupt
 Description  : Set true when MPU interrupt occurs
*/
volatile bool mpuInterrupt = false;

/* ==============================================================
 Function Name: dmpDataReady
 Input        : None
 Output       : None
 Logic        : Interrupt Service Routine (ISR) that sets a flag
                when MPU6050 DMP data is ready
 Example Call : Automatically triggered by hardware interrupt
 ============================================================== */
void dmpDataReady() {
  mpuInterrupt = true;
}

/* ==============================================================
                          PID PARAMETERS
   ============================================================== */

/*
 Variable Name: PID_MIN_LIMIT / PID_MAX_LIMIT
 Description  : Saturation limits for motor PWM output
 Range        : -255 to +255
*/
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT  255

/*
 Variable Name: PID_SAMPLE_TIME
 Description  : PID computation interval in milliseconds
 Range        : Typically 5–20 ms
*/
#define PID_SAMPLE_TIME 10

/*
 Variable Name: SETPOINT_PITCH_ANGLE_OFFSET
 Description  : Static mechanical offset for balance angle (degrees)
 Range        : Depends on COM shift, usually -15° to +15°
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
 Description  : Desired pitch angle for balancing (degrees)
*/
double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;

/*
 Variable Name: pitchGyroAngle
 Description  : Measured pitch angle from DMP (degrees)
*/
double pitchGyroAngle = 0;

/*
 Variable Name: pitchPIDOutput
 Description  : Output of pitch PID controller
 Range        : -255 to +255
*/
double pitchPIDOutput = 0;

/*
 տարինYaw variables handle yaw stabilization
*/
double setpointYawRate = 0;   // Desired yaw rate (deg/s)
double yawGyroRate = 0;       // Measured yaw rate
double yawPIDOutput = 0;      // Yaw correction output

/* ========================= PID GAINS ========================= */

/*
 Pitch PID controls balance (inner loop)
*/
#define PID_PITCH_KP 28
#define PID_PITCH_KI 0.8
#define PID_PITCH_KD 1.2

/*
 Yaw PID provides directional correction
*/
#define PID_YAW_KP 1
#define PID_YAW_KI 0.3
#define PID_YAW_KD 0

/*
 Pitch PID Object
 Input  : pitchGyroAngle
 Output : pitchPIDOutput
 Setpoint: setpointPitchAngle
*/
PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle,
             PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);

/*
 Yaw PID Object
*/
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate,
           PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

/* ==============================================================
                      MOTOR DRIVER CONFIG
   ============================================================== */

/*
 Motor A (Left Motor)
*/
int enableMotor1 = 6;     // PWM pin
int motor1Pin1  = A2;    // Direction pin
int motor1Pin2  = A3;    // Direction pin

/*
 Motor B (Right Motor)
*/
int enableMotor2 = 5;    // PWM pin
int motor2Pin1  = 9;     // Direction pin
int motor2Pin2  = 4;     // Direction pin

/* ==============================================================
 Function Name: setupMotors
 Input        : None
 Output       : None
 Logic        : Initializes motor control pins and stops motors
 Example Call : setupMotors();
 ============================================================== */
void setupMotors() {
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  rotateMotor(0, 0);
}

/* ==============================================================
 Function Name: setupPID
 Input        : None
 Output       : None
 Logic        : Configures PID limits, sampling time, and mode
 Example Call : setupPID();
 ============================================================== */
void setupPID() {
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME);
  pitchPID.SetMode(AUTOMATIC);

  yawPID.SetOutputLimits(-60, 60);
  yawPID.SetSampleTime(PID_SAMPLE_TIME);
  yawPID.SetMode(AUTOMATIC);
}

/* ==============================================================
 Function Name: setupMPU
 Input        : None
 Output       : None
 Logic        : Initializes MPU6050, loads DMP, applies calibration
 Example Call : setupMPU();
 ============================================================== */
void setupMPU() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // MPU calibration offsets
  mpu.setXAccelOffset(-4092);
  mpu.setYAccelOffset(687);
  mpu.setZAccelOffset(1430);
  mpu.setXGyroOffset(132);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(47);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

/* ==============================================================
                         ARDUINO MAIN
   ============================================================== */

void setup() {
  setupMotors();
  setupMPU();
  setupPID();
}

void loop() {

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    pitchGyroAngle = ypr[1] * 180.0 / M_PI;
    yawGyroRate    = gy.z;

    pitchPID.Compute();
    yawPID.Compute();

    int leftMotor  = pitchPIDOutput + yawPIDOutput;
    int rightMotor = pitchPIDOutput - yawPIDOutput;

    rotateMotor(leftMotor, rightMotor);

#ifdef PRINT_DEBUG_BUILD
    Serial.print("Pitch: ");
    Serial.print(pitchGyroAngle);
    Serial.print(" | PID: ");
    Serial.println(pitchPIDOutput);
#endif
  }
}

/* ==============================================================
 Function Name: rotateMotor
 Input        :
   speed1 → Left motor command (-255 to +255)
   speed2 → Right motor command (-255 to +255)
 Output       : None
 Logic        :
   • Determines motor direction
   • Applies dead-zone compensation
   • Outputs PWM to L298N driver
 Example Call : rotateMotor(120, -100);
 ============================================================== */
void rotateMotor(int speed1, int speed2)
{
  // Left Motor Direction
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  // Right Motor Direction (Inverted wiring)
  if (speed2 < 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }

  speed1 = constrain(abs(speed1) + MIN_ABSOLUTE_SPEED, 0, 255);
  speed2 = constrain(abs(speed2) + MIN_ABSOLUTE_SPEED, 0, 255);

  analogWrite(enableMotor1, speed1);
  analogWrite(enableMotor2, speed2);
}
