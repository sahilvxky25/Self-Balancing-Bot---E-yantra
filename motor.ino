const int inputPin1  = A2;    // Pin 15 of L293D IC
const int inputPin2  = A3;    // Pin 10 of L293D IC
                              //Motor B
const int inputPin3  = 9;    // Pin  7 of L293D IC
const int inputPin4  = 4;    // Pin  2 of L293D IC
int EN1 = 6;                 // Pin 1 of L293D IC
int EN2 = 5; 
void setup() {
    pinMode(EN1, OUTPUT);   // where the motor is connected to
   pinMode(EN2, OUTPUT);   // where the motor is connected to
   pinMode(inputPin1, OUTPUT);
   pinMode(inputPin2, OUTPUT);
   pinMode(inputPin3, OUTPUT);
   pinMode(inputPin4, OUTPUT);
   analogWrite(EN1, 100);      //sets the motors speed
   analogWrite(EN2, 100);

}

void loop() {
   delay(500);
   tone(12, 2000);
   delay(100);
   noTone(12);
   digitalWrite(inputPin1, HIGH);
   digitalWrite(inputPin2, LOW);
   digitalWrite(inputPin3, HIGH);
   digitalWrite(inputPin4, LOW);
   delay(200);
   digitalWrite(inputPin1, LOW);
   digitalWrite(inputPin2, HIGH);
   digitalWrite(inputPin3, LOW);
   digitalWrite(inputPin4, HIGH); 
   delay(200);

}
