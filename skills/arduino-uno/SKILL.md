---
name: arduino-uno
description: Arduino Uno R3 hardware platform specifications including ATmega328P pin layout, PWM capabilities, analog inputs, communication interfaces, and timing constraints. Reference this when generating code for Arduino Uno.
user-invocable: false
---

# Arduino Uno R3 Hardware Platform

## Core Specifications

- **MCU**: ATmega328P
- **Operating Voltage**: 5V
- **Input Voltage**: 7-12V (recommended), 6-20V (limits)
- **Clock Speed**: 16 MHz
- **Flash Memory**: 32 KB (0.5 KB used by bootloader)
- **SRAM**: 2 KB
- **EEPROM**: 1 KB

## GPIO Pin Capabilities

### Digital I/O Pins
**Available**: D0-D13 (14 pins)
- Can be INPUT, OUTPUT, or INPUT_PULLUP
- Each pin can source/sink up to 40 mA (20 mA recommended)
- Total current across all pins: 200 mA max

### PWM-Capable Pins (marked with ~)
**PWM Pins**: 3, 5, 6, 9, 10, 11
- 8-bit PWM (0-255 via `analogWrite()`)
- **Pins 5, 6**: ~980 Hz frequency
- **Pins 3, 9, 10, 11**: ~490 Hz frequency
- ⚠️ Pins 5 and 6 share a timer with `millis()` and `delay()`

### Analog Input Pins
**Analog Pins**: A0-A5 (6 pins)
- 10-bit resolution (0-1023)
- 0-5V input range
- Can also be used as digital I/O (pins 14-19)
- Reference voltage: AREF pin (default 5V)

### Interrupt-Capable Pins
**External Interrupts**: Pin 2 (INT0), Pin 3 (INT1)
- Triggered on: RISING, FALLING, CHANGE, LOW
- Use `attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)`
- Pin Change Interrupts available on all pins (more complex)

## Communication Interfaces

### UART (Serial)
**Pins**: RX=D0, TX=D1
- Connected to USB-to-Serial chip
- Baud rates: 300 to 115200 (higher rates unreliable)
- **Standard**: 9600 baud
- ⚠️ D0 and D1 should not be used while Serial communication is active

### I2C (Wire)
**Pins**: SDA=A4, SCL=A5
- Master or slave mode
- Standard (100 kHz) and Fast (400 kHz) modes
- Internal pull-up resistors available
- External 4.7kΩ pull-ups recommended for reliable operation

### SPI
**Pins**:
- MOSI=D11
- MISO=D12
- SCK=D13
- SS=D10 (user-defined, typically D10)

## Special Pin Functions

### Pin 13 (Built-in LED)
- Has built-in LED and 1kΩ series resistor
- LED is on when pin is HIGH
- May interfere with SPI (shares SCK)
- Good for basic testing without external components

### Pin 0 (RX)
- Serial receive
- Avoid using during Serial communication
- Can cause upload failures if externally loaded

### Pin 1 (TX)
- Serial transmit
- Avoid using during Serial communication
- Can cause upload failures if externally loaded

### AREF Pin
- Analog reference voltage for ADC
- Default: connected to 5V internally
- Can use external reference (1.1V internal, 5V, or external via AREF pin)
- ⚠️ Never connect voltage to AREF if using internal reference

### IOREF Pin
- Provides reference voltage (5V on Uno)
- Used by shields to detect board voltage level

## Pin Selection Best Practices

### For Digital Output (LED, Relay, etc.)
**Best choices**: 2, 4, 7, 8, 12, 13
**Avoid**: 0, 1 (Serial), 10-13 if using SPI

### For Digital Input (Button, Sensor)
**Best choices**: 2, 3, 4, 7, 8
**Recommended for interrupts**: 2, 3

### For PWM (Servo, Motor, LED dimming)
**Available**: 3, 5, 6, 9, 10, 11
**Best for servo**: 9, 10 (490 Hz is better for servos)
**Best for LED dimming**: 5, 6 (980 Hz reduces flicker)

### For Analog Input (Sensors)
**Available**: A0-A5
**Can also use as digital**: Refer to as pins 14-19 in code

### For I2C
**Fixed pins**: SDA=A4, SCL=A5
**Cannot be remapped** on Uno

### For SPI
**Fixed pins**: MOSI=11, MISO=12, SCK=13
**SS (Slave Select)**: User-defined, typically D10

## Timing Considerations

### millis() and delay()
- Resolution: 1 millisecond
- Based on Timer0
- ⚠️ Affected by PWM on pins 5 and 6

### micros() and delayMicroseconds()
- Resolution: 4 microseconds
- Also uses Timer0

### Timer Usage
- **Timer0**: millis(), delay(), PWM on pins 5 and 6
- **Timer1**: Servo library, PWM on pins 9 and 10
- **Timer2**: tone(), PWM on pins 3 and 11

### Servo Library
- Uses Timer1
- Disables PWM on pins 9 and 10
- Can control up to 12 servos (Uno has only 2 timers available)

## Power Specifications

### Per-Pin Current
- **Maximum**: 40 mA per pin
- **Recommended**: 20 mA per pin
- **Total**: 200 mA across all I/O pins

### Current-Limiting Resistors
For LEDs:
- Red LED: 220Ω to 1kΩ
- Blue/White LED: 100Ω to 220Ω
- Always use resistors to prevent pin damage

### Power Supply
- **USB**: 5V, up to 500 mA
- **DC Jack**: 7-12V (9V recommended), regulated to 5V
- **5V Pin**: Can supply 5V if powered via DC jack
- **3.3V Pin**: 50 mA maximum (from onboard regulator)

## Pin Conflict Checker

Before assigning a pin, verify:
1. ✅ Pin supports required function (PWM/Analog/Digital)
2. ✅ Not pins 0 or 1 if using Serial
3. ✅ Not pins 10-13 if using SPI (except for SPI itself)
4. ✅ Not A4 or A5 if using I2C (except for I2C itself)
5. ✅ Not already used by another peripheral
6. ✅ If using Servo library, PWM on pins 9 and 10 will be disabled

## Common Pin Assignments

### Safe General Purpose Pins
These are the safest pins for most projects:
- **D2, D3**: Good for interrupts (buttons, encoders)
- **D4, D7, D8**: Good for general digital I/O
- **D5, D6**: Best for high-frequency PWM (LED dimming)
- **D9, D10**: Best for servo control

### Pins to Avoid or Use Carefully
- **D0, D1**: Used by Serial (USB), avoid for general I/O
- **D13**: Has built-in LED and resistor, may affect sensitive circuits
- **A4, A5**: Used for I2C, can be repurposed but not recommended

## Hardware Constraints

### Memory Limitations
- **Flash**: 32 KB total, ~30.5 KB available
  - Use `F()` macro for string literals to save RAM
  - Use PROGMEM for large constant arrays

- **SRAM**: Only 2 KB
  - Avoid large arrays or strings
  - Use `String` class carefully (causes fragmentation)
  - Static allocation preferred over dynamic

- **EEPROM**: 1 KB
  - 100,000 write cycles per location
  - Use for configuration storage, not frequent logging

### Timing Limitations
- No floating-point unit (FPU) - float math is slow
- Integer math preferred
- 16 MHz clock limits processing speed

## Upload Process

### Reset Pin
- Automatically resets via DTR signal from USB
- Can be manually reset via button
- Reset required before upload

### Bootloader
- Occupies 0.5 KB of flash
- Allows USB upload without external programmer
- Timeout: ~1 second after reset
