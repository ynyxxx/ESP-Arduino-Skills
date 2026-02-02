---
name: esp32
description: ESP32 hardware platform specifications including GPIO pins, PWM capabilities, ADC/DAC, communication interfaces, and hardware constraints. Reference this when generating code for ESP32 or validating pin configurations.
user-invocable: false
---

# ESP32 DevKit V1 Hardware Platform

## Core Specifications

- **MCU**: ESP32-WROOM-32 (Dual-core Xtensa LX6)
- **Operating Voltage**: 3.3V (⚠️ NOT 5V tolerant)
- **Clock Speed**: 240 MHz (dual-core, adjustable)
- **Flash Memory**: 4 MB
- **SRAM**: 520 KB
- **ROM**: 448 KB
- **Built-in Peripherals**: WiFi 802.11 b/g/n, Bluetooth v4.2 BR/EDR and BLE

## GPIO Pin Capabilities

### Digital GPIO Pins
**Available**: GPIO 0-39 (40 pins total)

**Usable for general I/O**: 0-5, 12-19, 21-23, 25-27, 32-39

### PWM-Capable Pins
**All GPIO pins support PWM** via LEDC peripheral (16 independent channels, 0-15)
- Frequency range: 1 Hz to 40 MHz
- Resolution: 1-16 bits (configurable)
- **Recommended for PWM**: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27

### Analog Input (ADC)
**ADC1 (usable with WiFi)**: GPIO 32, 33, 34, 35, 36, 39
- 12-bit resolution (0-4095)
- 0-3.3V input range
- Attenuation configurable (0dB, 2.5dB, 6dB, 11dB)

**ADC2 (CANNOT use when WiFi active)**: GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27
- Same specs as ADC1
- ⚠️ **Critical**: ADC2 is disabled when WiFi is running

### Analog Output (DAC)
**DAC Pins**: GPIO 25, GPIO 26
- 8-bit resolution (0-255)
- 0-3.3V output range
- Use `dacWrite(pin, value)`

### Interrupt-Capable Pins
**All GPIO pins support interrupts**
- Rising edge, falling edge, both edges, low level, high level
- Use `attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)`

## Communication Interfaces

### UART (Serial)
**UART0**: RX=GPIO3, TX=GPIO1 (connected to USB)
- Default for Serial.println()
- Used by bootloader

**UART1**: RX=GPIO9, TX=GPIO10
- ⚠️ Connected to flash memory, not recommended for general use

**UART2**: RX=GPIO16, TX=GPIO17
- **Recommended** for external serial devices
- Can be remapped to other pins

### I2C (Wire)
**Default pins**: SDA=GPIO21, SCL=GPIO22
- Can be remapped to any GPIO pair
- Master or slave mode
- Standard (100 kHz) and Fast (400 kHz) modes

### SPI
**VSPI (default)**:
- MOSI=GPIO23
- MISO=GPIO19
- SCK=GPIO18
- SS=GPIO5

**HSPI**:
- MOSI=GPIO13
- MISO=GPIO12
- SCK=GPIO14
- SS=GPIO15

Both can be remapped to other pins.

## Pin Constraints and Special Functions

### ⚠️ Input-Only Pins
**GPIO 34, 35, 36, 39** are INPUT ONLY
- No internal pull-up/pull-down resistors
- Cannot be used with `pinMode(pin, OUTPUT)`
- Cannot be used with `pinMode(pin, INPUT_PULLUP)`

### ⚠️ Strapping Pins (Boot Mode)
These pins affect boot mode and should be avoided for critical functions:

**GPIO 0**:
- Must be HIGH during boot
- Has internal pull-up
- Boot button usually connected here
- Safe to use after boot

**GPIO 2**:
- Must be LOW or floating during boot
- Has internal pull-down
- Some boards have built-in LED here

**GPIO 5**:
- Controls SDIO slave timing
- Has internal pull-up

**GPIO 12**:
- Controls flash voltage (VDD_SDIO)
- Must be LOW during boot for 3.3V flash
- Has internal pull-down

**GPIO 15**:
- Must be HIGH during boot
- Controls boot message output
- Has internal pull-up

### Touch Sensing
**Touch-capable pins**: GPIO 0, 2, 4, 12, 13, 14, 15, 27, 32, 33
- Use `touchRead(pin)` to get capacitance value
- Threshold-based touch detection

### RTC GPIO (Deep Sleep Wake)
**RTC_GPIO pins**: 0, 2, 4, 12-15, 25-27, 32-39
- Can wake ESP32 from deep sleep
- Retain state during deep sleep

## Pin Selection Best Practices

### For Digital Output (LED, Relay, etc.)
**Best choices**: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
**Avoid**: Strapping pins (0, 2, 5, 12, 15), input-only pins (34-39)

### For Digital Input (Button, Sensor)
**Best choices**: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39
**Avoid**: Strapping pins if possible

### For PWM (Servo, Motor, LED dimming)
**Best choices**: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27
**Avoid**: Strapping pins, input-only pins

### For Analog Input (Sensors)
**Best choices**: 32, 33, 34, 35, 36, 39 (ADC1 - works with WiFi)
**Avoid**: ADC2 pins if using WiFi (0, 2, 4, 12-15, 25-27)

### For I2C
**Default**: SDA=21, SCL=22 (recommended to keep default)
**Alternative**: Any GPIO pair except input-only pins

### For SPI
**Recommended**: Use default VSPI pins (23, 19, 18, 5)
**Alternative**: HSPI or custom pins

## Power and Current

- **Max current per GPIO**: 40 mA (recommended 20 mA)
- **Max total current**: 200 mA across all pins
- **Operating voltage**: 3.3V logic level
- ⚠️ **Never connect 5V signals directly** - use level shifters

## Pin Conflict Checker

Before assigning a pin, verify:
1. ✅ Pin supports required function (INPUT/OUTPUT/PWM/ADC/etc.)
2. ✅ Not an input-only pin if output needed (avoid 34, 35, 36, 39)
3. ✅ Not a strapping pin if possible (avoid 0, 2, 5, 12, 15)
4. ✅ Not already used by another peripheral in your project
5. ✅ If using WiFi, avoid ADC2 pins for analog input

## Common Pin Assignments

### Safe General Purpose Pins
These are the safest pins for most projects:
- **GPIO 16, 17**: Often free, good for general I/O
- **GPIO 21, 22**: I2C default but can be repurposed
- **GPIO 23**: VSPI MOSI but can be repurposed
- **GPIO 25, 26**: Have DAC if needed
- **GPIO 32, 33**: Good for analog or digital

### Pins to Avoid
- **GPIO 6-11**: Connected to flash memory (SPI flash)
- **GPIO 34-39**: Input only
- **GPIO 1, 3**: UART0 (USB serial)

## Hardware Errata

- ADC readings can be affected by WiFi activity (use ADC1)
- Some boards have LED on GPIO 2 that can interfere
- Touch sensor values vary with board design
- Deep sleep current depends on which pins are used
