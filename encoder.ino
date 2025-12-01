#define encodPinAR 2   
#define encodPinBR 3   
#define encodPinAL 7   
#define encodPinBL 8   

volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;

int prevA_right = LOW;
int prevA_left  = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  prevA_right = digitalRead(enctodPinAR);
  prevA_left  = digitalRead(encodPinAL);
}

void loop() {
 int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {     
    if (digitalRead(encodPinBR) == HIGH) wheel_pulse_count_right++;
    else wheel_pulse_count_right--;
  }
  prevA_right = aR;
 int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {     
    if (digitalRead(encodPinBL) == HIGH) wheel_pulse_count_left++;
    else wheel_pulse_count_left--;
  }
  prevA_left = aL;
 static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 20) { 
    lastPrint = millis();
    Serial.print(wheel_pulse_count_left);
    Serial.print(" ; ");
    Serial.println(wheel_pulse_count_right);
  }
}
