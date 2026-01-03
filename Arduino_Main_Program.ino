#include "I2Cdev.h"
#include<Wire.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include<SoftwareSerial.h>
#include <digitalIOPerformance.h>                //library for faster pin R/W
//#include <Ultrasonic.h>

#define d_speed 1.5
#define d_dir 3     


#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 3

char content = 'P';
int MotorAspeed, MotorBspeed;
float MOTORSLACK_A = 40;                   // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B = 40;
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

MPU6050 mpu;

const int rxpin = 6;       //Bluetooth serial stuff
const int txpin = 5;
SoftwareSerial blue(rxpin, txpin);

//Ultrasonic ultrasonic(A0, A1);
//int distance;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



/*********Tune these 4 values for your BOT*********/
double setpoint; //set the value when the bot is perpendicular to ground using serial monitor.
double originalSetpoint;
//Read the project documentation on circuitdigest.com to learn how to set these values
#define Kp  10 //Set this first
#define Kd  0.6 //Set this secound
#define Ki  160 //Finally set this

#define RKp  50 //Set this first
#define RKd 4//Set this secound
#define RKi  300 //Finally set this
/******End of values setting*********/
double ysetpoint;
double yoriginalSetpoint;
double input, yinput, youtput, output, Buffer[3];

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID rot(&yinput, &youtput, &ysetpoint, RKp, RKi, RKd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup() {
  Serial.begin(115200);
  blue.begin(9600);
  blue.setTimeout(10);
  init_imu();           //initialiser le MPU6050
  initmot();            //initialiser les moteurs
  originalSetpoint = 176;  //consigne
  yoriginalSetpoint = 0.1;
  setpoint = originalSetpoint ;
  ysetpoint = yoriginalSetpoint ;
}



void loop() {
    getvalues();
    Bt_control();
    printval();
}







void init_imu() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  TWBR = 24;
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    rot.SetMode(AUTOMATIC);
    rot.SetSampleTime(10);
    rot.SetOutputLimits(-20, 20);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


void getvalues() {
  // if programming failed, don't try to do anything

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    new_pid();
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

    input = ypr[1] * 180 / M_PI + 180;
    yinput = ypr[0] * 180 / M_PI;
  }
}


void new_pid() {
  //Compute error
  pid.Compute();
  rot.Compute();
  // Convert PID output to motor control
  MotorAspeed = compensate_slack(youtput, output, 1);
  MotorBspeed = compensate_slack(youtput, output, 0);
  motorspeed(MotorAspeed, MotorBspeed);            //change speed
}
//Fast digitalWrite is implemented


void Bt_control() {
  if (blue.available()) {
    content = blue.read();
    if (content == 'F')
      setpoint = originalSetpoint - d_speed;//Serial.println(setpoint);}            //forward
    else if (content == 'B')
      setpoint = originalSetpoint + d_speed;//Serial.println(setpoint);}            //backward
    else if (content == 'L')
      ysetpoint = constrain((ysetpoint + yoriginalSetpoint - d_dir), -180, 180); //Serial.println(ysetpoint);}      //left
    else if (content == 'R')
      ysetpoint = constrain(ysetpoint + yoriginalSetpoint + d_dir, -180, 180); //Serial.println(ysetpoint);}        //right
    else if (content == 'S') {
      setpoint = originalSetpoint;
    }
  }
  else content = 'P';
}

void initmot() {
  //Initialise the Motor outpu pins
  pinMode (IN4, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN1, OUTPUT);

  //By default turn off both the motor
  analogWrite(IN4, LOW);
  analogWrite(IN3, LOW);
  analogWrite(IN2, LOW);
  analogWrite(IN1, LOW);
}

double compensate_slack(double yOutput, double Output, bool A) {
  // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  //yOutput is for left,right control
  if (A)
  {
    if (Output >= 0)
      Output = Output + MOTORSLACK_A - yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (Output >= 0)
      Output = Output + MOTORSLACK_B + yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_B + yOutput;
  }
  Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
  return Output;
}


void motorspeed(int MotorAspeed, int MotorBspeed) {

  // Motor A control
  if (MotorAspeed >= 0) {
    analogWrite(IN1, abs(MotorAspeed));
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, abs(MotorAspeed));
  }

  // Motor B control
  if (MotorBspeed >= 0) {
    analogWrite(IN3, abs(MotorBspeed));
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, abs(MotorBspeed));
  }
}
void printval()
{
  Serial.print(yinput); Serial.print("\t");
  Serial.print(yoriginalSetpoint); Serial.print("\t");
  Serial.print(ysetpoint); Serial.print("\t");
  Serial.print(youtput); Serial.print("\t"); Serial.print("\t");
  Serial.print(input); Serial.print("\t");
  Serial.print(originalSetpoint); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  Serial.print(output); Serial.print("\t"); Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.print("\t"); Serial.print(content); Serial.println("\t");
}
