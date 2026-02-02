#include <Servo.h>

Servo myServo;
const int servoPin = 9;  // PWM-capable pin

void setup() {
  Serial.begin(9600);

  myServo.attach(servoPin);
  myServo.write(90);  // Center position
  delay(500);

  Serial.println("Arduino Uno Servo initialized on Pin 9");
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
