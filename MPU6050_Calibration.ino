// ===============================================================
//  MPU6050 DMP INITIALIZATION & CALIBRATION SKETCH
//  Purpose : Verify MPU6050 connection, initialize DMP,
//            and perform accelerometer & gyroscope calibration
// ===============================================================

// ========================= LIBRARIES ===========================

// I2Cdev provides low-level I2C communication abstraction
#include "I2Cdev.h"

// MPU6050 driver with Digital Motion Processor (DMP) support
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev uses Arduino I2C backend
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ========================= GLOBAL OBJECTS =======================

/*
 Variable Name : mpu
 Description   : MPU6050 sensor object handling IMU + DMP
 Range         : Hardware interface object
*/
MPU6050 mpu;

/*
 Variable Name : devStatus
 Description   : Status returned by DMP initialization
                 0  → Success
                 >0 → Error (DMP load/config failure)
 Range         : 0–255
*/
uint8_t devStatus;

// ===============================================================
// Function Name : setup
// Input         : None
// Output        : None
// Logic         :
//   1. Initialize I2C bus
//   2. Initialize Serial communication
//   3. Initialize MPU6050
//   4. Verify sensor connection
//   5. Wait for user trigger (Serial input)
//   6. Load DMP firmware
//   7. Perform accelerometer & gyroscope calibration
//
// Example Call  : Automatically called once at boot
// ===============================================================
void setup() {

    /* ------------------ I2C INITIALIZATION ------------------ */
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();                 // Join I2C bus as master
        Wire.setClock(400000);        // Set I2C speed to 400 kHz (Fast Mode)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);   // Alternative fast I2C backend
    #endif

    /* ---------------- SERIAL INITIALIZATION ----------------- */
    /*
     Baud Rate : 115200 bps
     Reason    : Required for MPU6050 Teapot demo compatibility
     Note      : On 8 MHz or 3.3 V boards, use 38400 instead
    */
    Serial.begin(115200);

    /*
     Wait for Serial port to open
     Required for boards like Arduino Leonardo
     Other boards will continue immediately
    */
    while (!Serial);

    /* ------------------ MPU INITIALIZATION ------------------ */
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    /* ---------------- CONNECTION VERIFICATION --------------- */
    Serial.println(F("Testing device connections..."));
    Serial.println(
        mpu.testConnection()
        ? F("MPU6050 connection successful")
        : F("MPU6050 connection failed")
    );

    /* ---------------- USER TRIGGER -------------------------- */
    /*
     Waiting for user input ensures:
     - Serial monitor is open
     - User sees initialization messages
    */
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // Clear buffer
    while (!Serial.available());                 // Wait for input
    while (Serial.available() && Serial.read()); // Clear again

    /* ------------------ DMP INITIALIZATION ------------------ */
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* ------------------ CALIBRATION -------------------------- */
    /*
     Calibration improves sensor accuracy by estimating:
     - Accelerometer bias
     - Gyroscope bias
     Iterations (6) balance speed vs accuracy
    */
    if (devStatus == 0) 
    {
        Serial.println(F("Calibrating accelerometer..."));
        mpu.CalibrateAccel(6);

        Serial.println(F("Calibrating gyroscope..."));
        mpu.CalibrateGyro(6);

        /*
         Prints active offsets to Serial Monitor
         These values should be copied into
         production firmware as fixed offsets
        */
        Serial.println(F("Printing calibrated offsets:"));
        mpu.PrintActiveOffsets();
    }
    else
    {
        /*
         Non-zero devStatus indicates DMP load failure
         Common causes:
           - Incorrect wiring
           - Insufficient power
           - I2C timing issues
        */
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// ===============================================================
// Function Name : loop
// Input         : None
// Output        : None
// Logic         :
//   Empty loop since this sketch is meant only for
//   initialization, calibration, and offset extraction
//
// Example Call  : Continuously called after setup()
// ===============================================================
void loop() 
{
    // Intentionally left empty
}
