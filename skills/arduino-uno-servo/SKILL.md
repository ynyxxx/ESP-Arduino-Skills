---
name: arduino-uno-servo
description: Generate servo motor control code for Arduino Uno using standard Servo library. Handles PWM pin selection, Timer1 usage, and servo positioning for ATmega328P hardware.
allowed-tools: Read, Write
---

# Servo Motor Control for Arduino Uno

Generate complete servo motor control code for Arduino Uno using the standard Arduino Servo library.

## Quick Start

When user requests servo control, generate code that:
1. Uses **standard Servo.h library** (built-in)
2. Selects appropriate PWM-capable pin
3. Handles Timer1 usage properly
4. Implements requested behavior (sweep, position, interactive)

## Platform Requirements

**Refer to `arduino-uno` platform skill for:**
- Complete ATmega328P pin specifications
- PWM capabilities and timer assignments
- Memory constraints (2KB SRAM, 32KB Flash)
- Communication interfaces (UART, I2C, SPI)
- Current limitations and power requirements

**Key constraints for servo (from Arduino Uno platform):**
- ✅ PWM pins: 3, 5, 6, 9, 10, 11
- ⚠️ Servo library uses Timer1, which **disables PWM on pins 9 and 10**
- ⚠️ Operating voltage: 5V logic
- ⚠️ Max current per pin: 40 mA (recommended 20 mA)
- ⚠️ Limited RAM: Only 2KB SRAM (be memory-conscious)

## Arduino Framework Basics

### Program Structure
```cpp
void setup() {
  // Runs once at startup
  // Initialize pins, Serial, peripherals
}

void loop() {
  // Runs continuously after setup()
  // Main program logic
}
```

### Serial Communication
```cpp
Serial.begin(9600);           // Initialize Serial (standard baud rate)
Serial.println("Hello");      // Print with newline
Serial.print(value);          // Print without newline
if (Serial.available()) {     // Check if data available
  char c = Serial.read();     // Read one byte
}
```

### Timing Functions
```cpp
delay(1000);                  // Delay 1000ms (blocking)
delayMicroseconds(100);       // Delay 100μs (blocking)
unsigned long ms = millis();  // Milliseconds since boot (rolls over ~49 days)
unsigned long us = micros();  // Microseconds since boot (rolls over ~70 minutes)
```

### Math and Mapping
```cpp
int mapped = map(value, 0, 1023, 0, 255);  // Map range
int limited = constrain(value, 0, 255);    // Constrain to range
```

## Arduino Uno-Specific: Timer1 for Servo

**Critical understanding:** The Servo library uses Timer1 for precise pulse timing.

### Timer1 Impact
- **PWM on pins 9 and 10 is DISABLED** when servo is attached
- Cannot use `analogWrite(9, ...)` or `analogWrite(10, ...)`
- Timer1 no longer available for other timing functions
- This affects ALL servos, even if attached to other pins

### Why This Matters
Arduino Uno has three timers:
- **Timer0**: Used by `millis()`, `delay()`, PWM on pins 5 and 6
- **Timer1**: Used by Servo library, PWM on pins 9 and 10
- **Timer2**: Used by `tone()`, PWM on pins 3 and 11

Once you use the Servo library, Timer1 is dedicated to servo control.

## Pin Selection

### PWM-Capable Pins for Servo
Arduino Uno PWM pins: **3, 5, 6, 9, 10, 11**

### Recommended Servo Pins
**Best choices**: 9, 10 (490Hz PWM, better for servos)
**Acceptable**: 3, 11 (490Hz)
**Avoid**: 5, 6 (980Hz PWM, can cause servo jitter)

### Default Pin
If user doesn't specify: **Use Pin 9**

⚠️ **Remember**: PWM on pins 9 and 10 will be disabled once Servo library is used!

## Code Generation

### Basic Servo Sweep Example

```cpp
#include <Servo.h>

Servo myServo;

const int servoPin = 9;  // PWM-capable pin

void setup() {
  Serial.begin(9600);

  myServo.attach(servoPin);  // Attach servo to pin

  // Move to center position
  myServo.write(90);
  delay(500);

  Serial.println("Servo initialized");
}

void loop() {
  // Sweep from 0 to 180 degrees
  Serial.println("Sweeping 0 -> 180");
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);  // 15ms delay for smooth movement
  }

  delay(500);

  // Sweep from 180 to 0 degrees
  Serial.println("Sweeping 180 -> 0");
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }

  delay(500);
}
```

### Fixed Position Example

```cpp
#include <Servo.h>

Servo myServo;
const int servoPin = 9;

void setup() {
  Serial.begin(9600);

  myServo.attach(servoPin);

  // Set to specific position (0-180 degrees)
  myServo.write(90);  // Center position

  Serial.println("Servo at 90 degrees");
}

void loop() {
  // Servo holds position
  // No need to continuously write
}
```

### Interactive Control (Serial Input)

```cpp
#include <Servo.h>

Servo myServo;
const int servoPin = 9;

void setup() {
  Serial.begin(9600);

  myServo.attach(servoPin);
  myServo.write(90);

  Serial.println("Servo Control Ready");
  Serial.println("Enter angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    // Validate range
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("Servo moved to: ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid angle. Enter 0-180.");
    }

    // Clear remaining input
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
```

### Potentiometer Control

```cpp
#include <Servo.h>

Servo myServo;

const int servoPin = 9;
const int potPin = A0;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
}

void loop() {
  // Read potentiometer (0-1023)
  int potValue = analogRead(potPin);

  // Map to servo angle (0-180)
  int angle = map(potValue, 0, 1023, 0, 180);

  // Move servo
  myServo.write(angle);

  // Optional: print current position
  Serial.print("Pot: ");
  Serial.print(potValue);
  Serial.print(" -> Angle: ");
  Serial.println(angle);

  delay(15);  // Small delay for stability
}
```

### Button Control (Increment Position)

```cpp
#include <Servo.h>

Servo myServo;

const int servoPin = 9;
const int buttonPin = 2;

int currentAngle = 90;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(9600);

  pinMode(buttonPin, INPUT_PULLUP);
  myServo.attach(servoPin);
  myServo.write(currentAngle);

  Serial.println("Press button to increment servo position");
}

void loop() {
  int reading = digitalRead(buttonPin);

  // Debounce button
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      // Button pressed
      currentAngle += 15;
      if (currentAngle > 180) {
        currentAngle = 0;  // Wrap around
      }

      myServo.write(currentAngle);
      Serial.print("Angle: ");
      Serial.println(currentAngle);
    }
  }

  lastButtonState = reading;
}
```

### Multiple Servos (Limited by Timer)

```cpp
#include <Servo.h>

Servo servo1;
Servo servo2;

const int servo1Pin = 9;
const int servo2Pin = 10;

void setup() {
  Serial.begin(9600);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Different positions
  servo1.write(45);
  servo2.write(135);

  Serial.println("Multiple servos initialized");
}

void loop() {
  // Synchronized movement
  for (int pos = 0; pos <= 180; pos++) {
    servo1.write(pos);
    servo2.write(180 - pos);  // Mirror movement
    delay(15);
  }
  delay(1000);

  for (int pos = 180; pos >= 0; pos--) {
    servo1.write(pos);
    servo2.write(180 - pos);
    delay(15);
  }
  delay(1000);
}
```

## Servo Library Details

### Library Usage
```cpp
#include <Servo.h>  // Built-in library, no installation needed
```

### Key Functions

**attach()**: Connect servo to pin
```cpp
myServo.attach(pin);              // Default pulse width
myServo.attach(pin, min, max);    // Custom pulse width (μs)
// min: 544-1000μs (default 544)
// max: 2000-2400μs (default 2400)
```

**write()**: Set servo angle
```cpp
myServo.write(angle);  // 0-180 degrees
```

**writeMicroseconds()**: Set pulse width directly
```cpp
myServo.writeMicroseconds(1500);  // 1000-2000μs typical
```

**read()**: Get current angle
```cpp
int currentAngle = myServo.read();  // Returns 0-180
```

**attached()**: Check if servo is attached
```cpp
if (myServo.attached()) {
  // Servo is active
}
```

**detach()**: Release servo pin
```cpp
myServo.detach();  // Stops servo pulses, saves power
```

### Multiple Servos
- Arduino Uno can control **up to 12 servos** simultaneously
- All use Timer1
- Pins 9 and 10 lose PWM capability for ALL attached servos
- Use any digital pins, not just PWM pins

## Wiring

### Standard Servo Wiring

```
Servo Motor    Arduino Uno
-----------    -----------
Signal (Orange/Yellow) → Pin 9 (or other PWM pin)
VCC (Red)      → 5V
GND (Brown/Black) → GND
```

### Power Considerations

⚠️ **Important Power Notes**:

1. **Small servos (SG90, 9g micro)**:
   - Can power from Arduino 5V pin if:
     - Only 1-2 servos
     - Not under heavy load
     - Arduino powered via USB or wall adapter (not battery)

2. **Standard servos (SG5010, MG996R)**:
   - **MUST use external power**
   - Draw 100-500mA under load
   - Can damage Arduino or brown out if powered from board

3. **Proper external power setup**:
   ```
   External 5V PSU (1-2A)
   ├─ VCC → Servo VCC (Red)
   └─ GND → Servo GND (Brown) AND Arduino GND (common ground)

   Arduino Uno
   └─ Pin 9 → Servo Signal (Orange)
   ```

4. **Common ground is CRITICAL**:
   - Always connect Arduino GND to external power supply GND
   - Servo needs common ground for signal reference

5. **Capacitor recommendation**:
   - Add 100-470μF capacitor across servo power rails
   - Reduces voltage spikes and improves stability

## Code Patterns

### Smooth Movement

```cpp
void moveServoSmooth(int targetPos) {
  int currentPos = myServo.read();
  int step = (targetPos > currentPos) ? 1 : -1;

  for (int pos = currentPos; pos != targetPos; pos += step) {
    myServo.write(pos);
    delay(15);  // Adjust for speed
  }
}

// Usage
moveServoSmooth(180);  // Smooth move to 180°
```

### Non-blocking Movement

```cpp
int targetPosition = 90;
int currentPosition = 90;
unsigned long lastMoveTime = 0;
const int moveInterval = 15;  // ms between steps

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastMoveTime >= moveInterval) {
    if (currentPosition < targetPosition) {
      currentPosition++;
      myServo.write(currentPosition);
    } else if (currentPosition > targetPosition) {
      currentPosition--;
      myServo.write(currentPosition);
    }
    lastMoveTime = currentTime;
  }

  // Other code runs without blocking
}
```

### Detach to Save Power

```cpp
void setup() {
  myServo.attach(servoPin);
  myServo.write(90);
  delay(1000);  // Wait for servo to reach position
  myServo.detach();  // Saves power, servo stops holding position
}
```

### Custom Pulse Width (Calibration)

```cpp
// Some servos need custom pulse widths for full range
myServo.attach(9, 1000, 2000);  // Standard servo
myServo.attach(9, 544, 2400);   // Extended range
myServo.attach(9, 700, 2300);   // Custom calibration

// Test and adjust based on your servo's actual range
```

## Continuous Rotation Servos

```cpp
#include <Servo.h>

Servo continuousServo;

void setup() {
  continuousServo.attach(9);
}

void loop() {
  continuousServo.write(0);    // Full speed clockwise
  delay(2000);

  continuousServo.write(90);   // Stop
  delay(1000);

  continuousServo.write(180);  // Full speed counter-clockwise
  delay(2000);

  continuousServo.write(90);   // Stop
  delay(1000);
}
```

## Memory Considerations

The Servo library uses:
- ~500 bytes of flash memory
- Minimal RAM (a few bytes per servo object)

With Arduino Uno's limited 2KB RAM, avoid:
- Creating excessive servo objects
- Large delays in servo control loops
- Unnecessary String concatenation (use char arrays instead)

## Troubleshooting

### Servo Jitters or Doesn't Move Smoothly
- Use external power supply for standard servos
- Ensure stable 5V power (not battery near depletion)
- Add capacitor (100-470μF) across servo power
- Check for loose connections
- Avoid using pins 5 or 6 (980Hz PWM causes jitter)

### Servo Position Inaccurate
- Calibrate with `attach(pin, min, max)`
- Try `attach(9, 1000, 2000)` for standard servos
- Some cheap servos have poor accuracy

### Servo Doesn't Reach Full Range (0-180°)
- Adjust pulse width: `attach(9, 544, 2400)` for extended range
- Some servos are limited to ~160-170° actual range

### Multiple Servos Don't Work
- Ensure each servo has its own pin
- Use external power for 3+ servos
- Check common ground connection

### Arduino Resets When Servo Moves
- **Power issue**: Servo drawing too much current
- Use external 5V power supply
- Add capacitor across servo power rails

### PWM Doesn't Work on Pins 9 and 10
- **Expected behavior**: Servo library uses Timer1
- PWM on pins 9 and 10 is disabled when ANY servo is attached
- Use other PWM pins (3, 5, 6, 11) for LED dimming, etc.

## Performance Notes

- Servo library uses interrupts for precise timing
- Minimal CPU overhead
- Update rate: ~50Hz (every 20ms)
- Position resolution: 1 degree (0-180)

## Additional Resources

See [resources/templates/](resources/templates/) for more examples.
