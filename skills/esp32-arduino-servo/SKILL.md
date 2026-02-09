---
name: esp32-arduino-servo
description: Generate servo motor control code for ESP32 using Arduino framework with ESP32Servo library. Handles PWM pin selection, LEDC configuration, and servo positioning for ESP32 hardware.
allowed-tools: Read, Write
---

# Servo Motor Control for ESP32 (Arduino Framework)

Generate complete servo motor control code for ESP32 using the Arduino framework.

## Quick Start

When user requests servo control, generate code that:
1. Uses **ESP32Servo library** (not standard Servo.h)
2. Selects appropriate PWM-capable pin
3. Configures LEDC peripheral correctly
4. Implements requested behavior (sweep, position, interactive)

## Platform Requirements

**Refer to `esp32` platform skill for:**
- Complete GPIO specifications
- Hardware constraints and strapping pins
- Power requirements (3.3V logic)
- ADC, DAC, and communication interface details

**Key constraints for servo (from ESP32 platform):**
- ✅ All GPIO pins support PWM via LEDC peripheral
- ❌ Avoid input-only pins (34, 35, 36, 39) - cannot be used as outputs
- ❌ Avoid strapping pins if possible (0, 2, 5, 12, 15) - affect boot mode
- ⚠️ Operating voltage: 3.3V logic (not 5V tolerant)
- ⚠️ Max current per GPIO: 40 mA (recommended 20 mA)

## Arduino Framework Basics

### Program Structure
```cpp
void setup() {
  // Runs once at startup
  // Initialize pins, Serial, servo, etc.
}

void loop() {
  // Runs repeatedly after setup()
  // Main program logic
}
```

### Serial Communication
```cpp
Serial.begin(115200);         // Initialize Serial (typical ESP32 baud rate)
Serial.println("Hello");      // Print with newline
Serial.print(value);          // Print without newline
```

### Timing Functions
```cpp
delay(1000);                  // Delay 1000ms (blocking)
delayMicroseconds(100);       // Delay 100μs (blocking)
unsigned long ms = millis();  // Milliseconds since boot
unsigned long us = micros();  // Microseconds since boot
```

## ESP32-Specific: LEDC PWM

**Critical for servo:** ESP32 uses LEDC (LED Control) peripheral, not traditional PWM timers.

### LEDC Features
- 16 independent channels (0-15)
- Frequency range: 1 Hz to 40 MHz
- Resolution: 1-16 bits (configurable)
- Any GPIO pin can be assigned to any channel

### Why This Matters for Servo
The ESP32Servo library abstracts LEDC complexity, but understanding the underlying mechanism helps:
- Servo signals require precise 50Hz PWM
- Pulse width controls servo position (typically 500-2400μs)
- LEDC automatically manages the timing

**You don't need to configure LEDC manually** - the ESP32Servo library handles it!

## Pin Selection

### Recommended Servo Pins
**Best choices**: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27
**Avoid**:
- 0, 2, 5, 12, 15 (strapping pins - affect boot)
- 34, 35, 36, 39 (input-only pins)
- 6-11 (connected to flash memory)
- 1, 3 (UART0 - USB serial)

### Default Pin
If user doesn't specify: **Use GPIO 18**

## Code Generation

### Basic Servo Sweep Example

```cpp
#include <ESP32Servo.h>

Servo myServo;

// Pin configuration
const int servoPin = 18;  // PWM-capable pin

void setup() {
  Serial.begin(115200);

  // ESP32Servo specific configuration
  myServo.setPeriodHertz(50);           // Standard 50Hz servo
  myServo.attach(servoPin, 500, 2400);  // Attach with pulse width range

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
#include <ESP32Servo.h>

Servo myServo;
const int servoPin = 18;

void setup() {
  Serial.begin(115200);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);

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
#include <ESP32Servo.h>

Servo myServo;
const int servoPin = 18;

void setup() {
  Serial.begin(115200);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
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

### Multiple Servos

```cpp
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;

const int servo1Pin = 18;
const int servo2Pin = 19;

void setup() {
  Serial.begin(115200);

  // Initialize servos on different LEDC channels (automatic)
  servo1.setPeriodHertz(50);
  servo1.attach(servo1Pin, 500, 2400);

  servo2.setPeriodHertz(50);
  servo2.attach(servo2Pin, 500, 2400);

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
}
```

## Important ESP32Servo Library Details

### Library Installation
```
// Arduino IDE: Tools > Manage Libraries > Search "ESP32Servo"
// PlatformIO: lib_deps = ESP32Servo
```

### Key Differences from Standard Servo Library

1. **Must call `setPeriodHertz()` before `attach()`**
   ```cpp
   myServo.setPeriodHertz(50);  // 50Hz for standard servos
   ```

2. **Attach with pulse width range**
   ```cpp
   myServo.attach(pin, minUs, maxUs);
   // minUs: min pulse width (default 544, standard 500-1000)
   // maxUs: max pulse width (default 2400, standard 2000-2400)
   ```

3. **Uses LEDC channels automatically**
   - ESP32Servo manages LEDC channels internally
   - Supports up to 16 servos (16 LEDC channels)

### Typical Servo Pulse Widths

| Servo Type | Min (0°) | Max (180°) |
|------------|----------|------------|
| Standard   | 1000μs   | 2000μs     |
| Extended   | 500μs    | 2400μs     |
| Continuous | 1000μs   | 2000μs     |

**Recommended**: Use `attach(pin, 500, 2400)` for maximum range

### Continuous Rotation Servos

```cpp
myServo.setPeriodHertz(50);
myServo.attach(servoPin, 1000, 2000);

// Continuous servos use position as speed
myServo.write(90);   // Stop
myServo.write(0);    // Full speed one direction
myServo.write(180);  // Full speed other direction
```

## Wiring

### Standard Servo Wiring

```
Servo Motor    ESP32
-----------    -----
Signal (Orange/Yellow) → GPIO [PIN] (e.g., 18)
VCC (Red)      → 5V (external power recommended)
GND (Brown/Black) → GND
```

### Power Considerations

⚠️ **Important Power Notes**:

1. **Small servos (SG90, 9g micro)**: Can power from ESP32 5V pin if:
   - Only 1-2 servos
   - Not under heavy load
   - ESP32 powered via USB (500mA available)

2. **Standard servos (SG5010, MG996R)**: **MUST use external power**
   - Draw 100-500mA under load
   - Can damage ESP32 if powered from board
   - Use external 5V power supply (1-2A)
   - **Connect grounds together** (ESP32 GND to external PSU GND)

3. **Proper external power setup**:
   ```
   External 5V PSU
   ├─ VCC → Servo VCC (Red)
   └─ GND → Servo GND (Brown) AND ESP32 GND (common ground)

   ESP32
   └─ GPIO 18 → Servo Signal (Orange)
   ```

## Code Patterns

### Smooth Acceleration

```cpp
void moveServoSmooth(Servo &servo, int targetPos, int currentPos) {
  int step = (targetPos > currentPos) ? 1 : -1;

  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(10);  // Adjust for speed
  }
}

// Usage
int currentPosition = 90;
moveServoSmooth(myServo, 180, currentPosition);
currentPosition = 180;
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
  delay(500);  // Wait for servo to reach position
  myServo.detach();  // Saves power, servo stops holding position
}
```

## Required Libraries

```cpp
#include <ESP32Servo.h>  // ESP32-specific servo library
```

**Installation**:
- Arduino IDE: Library Manager → "ESP32Servo" by Kevin Harrington
- PlatformIO: Add to `platformio.ini`:
  ```ini
  lib_deps = ESP32Servo
  ```

## Troubleshooting

### Servo Jitters or Doesn't Move Smoothly
- Ensure stable power supply (use external 5V for standard servos)
- Check for loose wiring
- Add 100-470μF capacitor across servo power and ground
- Ensure common ground between ESP32 and external power

### Servo Position Inaccurate
- Calibrate pulse width: Try different values in `attach(pin, min, max)`
- Standard servos: `attach(pin, 1000, 2000)`
- Extended range: `attach(pin, 500, 2400)`

### Multiple Servos Interfere
- ESP32Servo automatically manages LEDC channels
- Ensure each servo uses different GPIO pin
- Use external power for multiple servos

### Servo Draws Too Much Current
- **Never** power multiple servos from ESP32 5V pin
- Use external 5V power supply (1-2A minimum)
- Connect ESP32 GND to external PSU GND (common ground required)

## Additional Resources

See [resources/templates/](resources/templates/) for more examples.
