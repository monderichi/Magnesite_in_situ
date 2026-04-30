const int valvePin = 7;

void setup() {
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW); // Ensure relay is OFF initially (normally open)
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(valvePin, HIGH); // Open valve / Close relay
    } else if (command == '0') {
      digitalWrite(valvePin, LOW); // Close valve / Open relay
    }
  }
}
