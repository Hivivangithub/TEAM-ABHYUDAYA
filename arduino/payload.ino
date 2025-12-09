// payload.ino - very simple demo code for servo dropper (upload to Arduino)
#include <Servo.h>
Servo lockServo;
const int SERVO_PIN = 9;

void setup() {
  Serial.begin(115200);
  lockServo.attach(SERVO_PIN);
  servo_lock();
  Serial.println("Payload Arduino ready");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'o') {
      servo_release();
      Serial.println("Released");
    }
    if (c == 'c') {
      servo_lock();
      Serial.println("Locked");
    }
  }
  delay(100);
}

void servo_lock() {
  lockServo.write(0); // locked
}

void servo_release() {
  lockServo.write(90); // released
}
