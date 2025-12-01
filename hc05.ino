int count = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("Bluetooth is ready!");
}

void loop() {

  if (Serial.available()) {
    char data = Serial.read();
    Serial.print("Received: ");
    Serial.println(data);
  }
}
