#include <ESP32Servo.h>

Servo myServo;
const int servoPin = 18;  // GPIO 18 (PWM-capable)

void setup() {
  Serial.begin(115200);

  // ESP32-specific servo configuration
  myServo.setPeriodHertz(50);           // 50Hz for standard servos
  myServo.attach(servoPin, 500, 2400);  // Attach with pulse width range

  myServo.write(90);  // Center position
  delay(500);

  Serial.println("ESP32 Servo initialized on GPIO 18");
}

void loop() {
  // Sweep 0 to 180
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);
  }
  delay(500);

  // Sweep 180 to 0
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }
  delay(500);
}
