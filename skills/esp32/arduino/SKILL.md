---
name: esp32-arduino
description: Arduino framework for ESP32 platform. Covers Arduino core API, ESP32-specific libraries, setup/loop structure, and framework-specific best practices for ESP32 development.
user-invocable: false
---

# Arduino Framework for ESP32

## Overview

The Arduino framework for ESP32 provides a familiar Arduino API while leveraging ESP32's advanced features. It's built on top of ESP-IDF but abstracts complexity.

## Framework Basics

### Program Structure
```cpp
void setup() {
  // Runs once at startup
  // Initialize pins, Serial, WiFi, etc.
}

void loop() {
  // Runs repeatedly after setup()
  // Main program logic
}
```

### Core Functions

**Digital I/O**:
```cpp
pinMode(pin, INPUT);          // Configure as input
pinMode(pin, OUTPUT);         // Configure as output
pinMode(pin, INPUT_PULLUP);   // Input with internal pull-up

digitalWrite(pin, HIGH);      // Set pin high (3.3V)
digitalWrite(pin, LOW);       // Set pin low (0V)

int value = digitalRead(pin); // Read digital value (HIGH/LOW)
```

**Analog Input**:
```cpp
int value = analogRead(pin);  // Read ADC (0-4095, 12-bit)
// Use ADC1 pins (32,33,34,35,36,39) if WiFi is active
```

**PWM Output (uses LEDC)**:
```cpp
// ESP32 requires LEDC setup (not standard analogWrite)
ledcSetup(channel, frequency, resolution);  // channel: 0-15
ledcAttachPin(pin, channel);
ledcWrite(channel, dutyCycle);

// Some libraries provide analogWrite() wrapper
analogWrite(pin, value);  // 0-255 (if library supports)
```

**Serial Communication**:
```cpp
Serial.begin(115200);         // Initialize Serial
Serial.println("Hello");      // Print with newline
Serial.print(value);          // Print without newline
Serial.available();           // Check if data available
Serial.read();                // Read one byte
```

**Timing**:
```cpp
delay(1000);                  // Delay 1000ms (blocking)
delayMicroseconds(100);       // Delay 100μs (blocking)
unsigned long ms = millis();  // Milliseconds since boot
unsigned long us = micros();  // Microseconds since boot
```

## ESP32-Specific Arduino Libraries

### WiFi
```cpp
#include <WiFi.h>

WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
  delay(500);
}
Serial.println(WiFi.localIP());
```

### Bluetooth Serial
```cpp
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
SerialBT.begin("ESP32_BT");
SerialBT.println("Hello Bluetooth");
```

### Web Server
```cpp
#include <WebServer.h>

WebServer server(80);
server.on("/", handleRoot);
server.begin();
server.handleClient();  // In loop()
```

### Preferences (Non-volatile storage)
```cpp
#include <Preferences.h>

Preferences preferences;
preferences.begin("my-app", false);  // namespace, read-only
preferences.putInt("counter", 0);
int value = preferences.getInt("counter", 0);
preferences.end();
```

### SPIFFS (File System)
```cpp
#include <SPIFFS.h>

SPIFFS.begin();
File file = SPIFFS.open("/data.txt", "r");
String content = file.readString();
file.close();
```

## ESP32 Arduino-Specific Features

### PWM via LEDC
ESP32 doesn't use traditional PWM timers. Instead, use LEDC:

```cpp
// Setup
const int ledPin = 16;
const int pwmChannel = 0;
const int freq = 5000;      // 5 kHz
const int resolution = 8;   // 8-bit (0-255)

ledcSetup(pwmChannel, freq, resolution);
ledcAttachPin(ledPin, pwmChannel);

// Use
ledcWrite(pwmChannel, 128);  // 50% duty cycle
```

### Servo Control (ESP32Servo Library)
```cpp
#include <ESP32Servo.h>  // NOT standard Servo.h

Servo myServo;
myServo.setPeriodHertz(50);        // Standard 50Hz servo
myServo.attach(servoPin, 500, 2400);  // pin, min, max pulse width
myServo.write(90);  // Position 0-180
```

### Touch Sensing
```cpp
int touchValue = touchRead(T0);  // T0 = GPIO4
// Lower value = touched
if (touchValue < 20) {
  Serial.println("Touch detected");
}
```

### DAC Output
```cpp
dacWrite(25, 128);  // GPIO25, value 0-255
// Outputs ~1.65V (128/255 * 3.3V)
```

### Multi-core Tasks
```cpp
void Task1(void *parameter) {
  while(true) {
    // Task code
    vTaskDelay(10);
  }
}

void setup() {
  xTaskCreatePinnedToCore(
    Task1,           // Function
    "Task1",         // Name
    10000,           // Stack size
    NULL,            // Parameters
    1,               // Priority
    NULL,            // Task handle
    0                // Core (0 or 1)
  );
}
```

## Library Ecosystem

### Core Libraries (Built-in)
- **WiFi**: WiFi connectivity
- **WiFiClient**: TCP client
- **WiFiServer**: TCP server
- **WebServer**: HTTP server
- **HTTPClient**: HTTP client requests
- **BluetoothSerial**: Bluetooth classic serial
- **Wire**: I2C communication
- **SPI**: SPI communication
- **Preferences**: NVS storage
- **SPIFFS**: File system
- **SD**: SD card support

### Popular Third-Party Libraries
- **ESP32Servo**: Servo motor control (replaces Servo.h)
- **PubSubClient**: MQTT client
- **ArduinoJson**: JSON parsing
- **Adafruit_GFX**: Graphics for displays
- **TFT_eSPI**: Fast TFT display driver
- **AsyncTCP**: Asynchronous TCP
- **ESPAsyncWebServer**: Async web server

## Important Differences from Arduino

### 1. Voltage Level
- **ESP32**: 3.3V logic
- **Arduino Uno**: 5V logic
- ⚠️ Never connect 5V signals directly to ESP32

### 2. PWM
- **ESP32**: Use LEDC (16 channels, any pin, any frequency)
- **Arduino**: Use analogWrite() (6 pins, fixed frequency)

### 3. Analog Input
- **ESP32**: 12-bit (0-4095), ADC2 conflicts with WiFi
- **Arduino**: 10-bit (0-1023), no WiFi conflict

### 4. Interrupts
- **ESP32**: All GPIO pins support interrupts
- **Arduino Uno**: Only pins 2 and 3

### 5. Serial Ports
- **ESP32**: 3 hardware UARTs (Serial, Serial1, Serial2)
- **Arduino Uno**: 1 hardware UART (Serial)

### 6. Servo Library
- **ESP32**: Must use ESP32Servo library (not standard Servo)
- **Arduino**: Standard Servo library works

## Best Practices

### Memory Management
```cpp
// Prefer stack allocation
char buffer[256];

// Avoid String class for frequent operations
String str = "Hello";  // Can cause fragmentation

// Use const char* or std::string for ESP32
const char* str = "Hello";
```

### WiFi Power Management
```cpp
WiFi.mode(WIFI_STA);           // Station mode
WiFi.setSleep(false);          // Disable sleep for stability
WiFi.setAutoReconnect(true);   // Auto reconnect
```

### Task Delays
```cpp
// In loop()
delay(10);  // OK for simple programs

// In FreeRTOS tasks
vTaskDelay(pdMS_TO_TICKS(10));  // Better for multitasking
```

### Pin Selection
```cpp
// Always check parent esp32 platform skill for:
// - Input-only pins (34, 35, 36, 39)
// - Strapping pins (0, 2, 5, 12, 15)
// - ADC1 vs ADC2 for WiFi compatibility
```

## Build Configuration

### Board Selection (Arduino IDE)
- **Board**: ESP32 Dev Module (or specific variant)
- **Upload Speed**: 921600 (or 115200 for reliability)
- **Flash Frequency**: 80MHz
- **Partition Scheme**: Default 4MB with spiffs

### PlatformIO Configuration
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

lib_deps =
    ESP32Servo
    PubSubClient
    ArduinoJson
```

## Common Patterns

### WiFi Connection
```cpp
void setup() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
}
```

### Non-blocking Delays
```cpp
unsigned long previousMillis = 0;
const long interval = 1000;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Do something every second
  }
}
```

### Debouncing
```cpp
const int buttonPin = 2;
int lastState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Stable state
    if (reading == LOW) {
      // Button pressed
    }
  }
  lastState = reading;
}
```

## Troubleshooting

### Upload Issues
- Press and hold BOOT button during upload
- Reduce upload speed to 115200
- Check USB cable (use data cable, not charge-only)

### WiFi Not Working
- Check ADC2 pin usage (ADC2 disabled when WiFi active)
- Verify 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- Disable WiFi sleep: `WiFi.setSleep(false)`

### Crash/Reset Loops
- Check power supply (ESP32 needs stable 3.3V, 500mA+)
- Avoid strapping pins during boot
- Check serial monitor for panic messages

## Reference

- Arduino ESP32 Core: https://github.com/espressif/arduino-esp32
- API Documentation: https://docs.espressif.com/projects/arduino-esp32/
