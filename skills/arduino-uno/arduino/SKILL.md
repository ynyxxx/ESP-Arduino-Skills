---
name: arduino-uno-arduino
description: Standard Arduino framework for Arduino Uno platform. Covers core Arduino API, standard libraries, and Uno-specific programming patterns.
user-invocable: false
---

# Arduino Framework for Arduino Uno

## Overview

This is the original Arduino framework on the native Arduino Uno platform. Provides the standard Arduino API with ATmega328P-specific implementations.

## Framework Basics

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

### Core Functions

**Digital I/O**:
```cpp
pinMode(pin, INPUT);          // Configure as input
pinMode(pin, OUTPUT);         // Configure as output
pinMode(pin, INPUT_PULLUP);   // Input with 20-50kΩ pull-up

digitalWrite(pin, HIGH);      // Set pin high (5V)
digitalWrite(pin, LOW);       // Set pin low (0V)

int value = digitalRead(pin); // Read digital value (HIGH/LOW)
```

**Analog Input**:
```cpp
int value = analogRead(A0);   // Read ADC (0-1023, 10-bit)
// Takes ~100μs per reading
// Returns 0-1023 for 0-5V input range
```

**PWM Output**:
```cpp
analogWrite(pin, value);      // PWM output (0-255)
// Only works on pins: 3, 5, 6, 9, 10, 11
// Pins 5,6: ~980Hz
// Pins 3,9,10,11: ~490Hz
```

**Serial Communication**:
```cpp
Serial.begin(9600);           // Initialize Serial (baud rate)
Serial.println("Hello");      // Print with newline
Serial.print(value);          // Print without newline
if (Serial.available()) {     // Check if data available
  char c = Serial.read();     // Read one byte
}
```

**Timing**:
```cpp
delay(1000);                  // Delay 1000ms (blocking)
delayMicroseconds(100);       // Delay 100μs (blocking)
unsigned long ms = millis();  // Milliseconds since boot (rolls over ~49 days)
unsigned long us = micros();  // Microseconds since boot (rolls over ~70 minutes)
```

**Math and Mapping**:
```cpp
int mapped = map(value, 0, 1023, 0, 255);  // Map range
int limited = constrain(value, 0, 255);    // Constrain to range
int minimum = min(a, b);
int maximum = max(a, b);
int absolute = abs(value);
```

**Random Numbers**:
```cpp
randomSeed(analogRead(0));    // Seed with noise from unconnected analog pin
int r = random(100);          // Random 0-99
int r = random(50, 100);      // Random 50-99
```

## Standard Libraries

### Servo
```cpp
#include <Servo.h>

Servo myServo;
myServo.attach(9);            // Attach to PWM pin
myServo.write(90);            // Set position (0-180 degrees)
int pos = myServo.read();     // Read current position
myServo.detach();             // Release pin
```

**Notes**:
- Disables PWM on pins 9 and 10 (uses Timer1)
- Can control up to 12 servos (limited by available timers)
- Pulse width: 1000-2000μs (default), customizable

### Wire (I2C)
```cpp
#include <Wire.h>

// Master mode
Wire.begin();                 // Join as master
Wire.beginTransmission(0x27); // Start transmission to slave
Wire.write(byte);             // Queue byte for transmission
Wire.endTransmission();       // Send queued bytes

Wire.requestFrom(0x27, 1);    // Request 1 byte from slave
if (Wire.available()) {
  char c = Wire.read();
}

// Slave mode
Wire.begin(address);          // Join as slave
Wire.onReceive(receiveEvent); // Register receive callback
Wire.onRequest(requestEvent); // Register request callback
```

**Pins**: SDA=A4, SCL=A5 (fixed, cannot change)

### SPI
```cpp
#include <SPI.h>

SPI.begin();                  // Initialize SPI
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
digitalWrite(SS, LOW);        // Select slave
byte response = SPI.transfer(data);
digitalWrite(SS, HIGH);       // Deselect slave
SPI.endTransaction();
```

**Pins**: MOSI=11, MISO=12, SCK=13, SS=10 (SS user-defined)

### SoftwareSerial
```cpp
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);  // RX, TX
mySerial.begin(9600);
mySerial.println("Hello");
```

**Notes**:
- Allows serial on any digital pins
- Not as reliable as hardware serial
- Max baud rate: ~57600 (lower more reliable)
- Only one SoftwareSerial can receive at a time

### EEPROM
```cpp
#include <EEPROM.h>

EEPROM.write(address, value);     // Write byte (0-255)
byte value = EEPROM.read(address); // Read byte
EEPROM.update(address, value);     // Write only if different (saves wear)

// For larger data
EEPROM.put(address, data);         // Write any data type
EEPROM.get(address, data);         // Read any data type
```

**Notes**:
- 1024 bytes (addresses 0-1023)
- 100,000 write cycles per location
- Use `update()` instead of `write()` to extend lifetime

### LiquidCrystal (LCD)
```cpp
#include <LiquidCrystal.h>

// LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

lcd.begin(16, 2);                 // 16 cols, 2 rows
lcd.print("Hello");
lcd.setCursor(0, 1);              // Col 0, Row 1
lcd.clear();
lcd.noDisplay();  / lcd.display();
lcd.noCursor();   / lcd.cursor();
lcd.noBlink();    / lcd.blink();
```

## Interrupts

### External Interrupts
```cpp
attachInterrupt(digitalPinToInterrupt(2), ISR_function, RISING);
// Pin 2 or 3 only
// Modes: LOW, CHANGE, RISING, FALLING

void ISR_function() {
  // Keep ISR short and fast
  // No delay(), Serial, or heavy operations
  // Use volatile variables
}

detachInterrupt(digitalPinToInterrupt(2));
```

### Pin Change Interrupts
```cpp
// More complex, can use any pin
// Requires direct register manipulation
// Not covered in basic Arduino framework
```

## Timer Usage

### Timer0 (8-bit)
- Used by: `millis()`, `delay()`, PWM pins 5 and 6
- Interrupts every 1ms for timekeeping
- Modifying Timer0 affects timing functions

### Timer1 (16-bit)
- Used by: Servo library, PWM pins 9 and 10
- Most flexible timer
- Can be used for custom timing

### Timer2 (8-bit)
- Used by: `tone()`, PWM pins 3 and 11
- Can be used for custom timing

## Memory Management

### Flash Memory (Program Space)
```cpp
// Store strings in flash instead of RAM
Serial.println(F("This string stays in flash"));

// Store arrays in flash
const PROGMEM uint16_t data[] = {1, 2, 3, 4};
uint16_t value = pgm_read_word(&data[0]);
```

### SRAM (2KB only!)
```cpp
// Avoid large arrays
int bigArray[100];  // Uses 200 bytes!

// Avoid String class
String str = "Hello";  // Causes fragmentation

// Prefer char arrays
char str[20] = "Hello";

// Check free RAM
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
```

## Best Practices

### Non-blocking Code
```cpp
// Bad - blocks everything
delay(1000);

// Good - non-blocking
unsigned long previousMillis = 0;
const long interval = 1000;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Do something every second
  }
  // Other code runs freely
}
```

### Button Debouncing
```cpp
const int buttonPin = 2;
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void loop() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        // Button pressed
      }
    }
  }

  lastButtonState = reading;
}
```

### Efficient Serial
```cpp
// Wait for Serial only if needed
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial (only needed for native USB boards)
  }
}

// Avoid String concatenation
String msg = "Value: " + String(value);  // Bad - uses lots of RAM

char msg[32];
sprintf(msg, "Value: %d", value);  // Better
Serial.println(msg);
```

### Power Saving
```cpp
#include <avr/sleep.h>
#include <avr/power.h>

set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_enable();
power_adc_disable();   // Disable ADC
power_spi_disable();   // Disable SPI
// ... disable other peripherals
sleep_mode();          // Enter sleep
```

## Build Configuration

### Arduino IDE
- **Board**: Arduino Uno
- **Processor**: ATmega328P
- **Port**: COM3 (Windows) or /dev/ttyACM0 (Linux)

### PlatformIO
```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 9600

lib_deps =
    Servo
    LiquidCrystal
```

## Common Patterns

### LED Blink
```cpp
const int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}
```

### Analog Sensor Reading
```cpp
void loop() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  Serial.println(voltage);
  delay(100);
}
```

### Multiple LEDs
```cpp
int ledPins[] = {2, 3, 4, 5, 6};
int numLeds = 5;

void setup() {
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}
```

## Limitations

- **No floating point unit**: Float math is slow (software emulation)
- **Limited RAM**: Only 2KB, careful memory management required
- **Single core**: No multithreading
- **No WiFi/Bluetooth**: Use external modules (ESP8266, HC-05, etc.)
- **Fixed pins**: I2C, SPI pins cannot be remapped
- **Limited interrupts**: Only 2 external interrupt pins

## Troubleshooting

### Upload Errors
- Check USB cable (use data cable, not charge-only)
- Close Serial Monitor before upload
- Press reset button before upload if needed
- Check board and port selection

### Program Not Running
- Upload includes bootloader delay (~1 second)
- Check power supply (USB or external)
- Verify code compiles without errors

### Unexpected Behavior
- Check for memory issues (RAM exhaustion)
- Verify pin numbers match physical connections
- Check for floating pins (use INPUT_PULLUP)
- Verify timing constraints (delay vs millis)
