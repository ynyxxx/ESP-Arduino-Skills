# Changelog

## [0.1.0] - Initial Release

### Migrated from esp-arduino-agent

This plugin was created by extracting and reorganizing hardware knowledge from the `esp-arduino-agent` project into a reusable, framework-agnostic skills library.

### Platforms

#### ESP32
- **Source**: `src/agent/skillsets.py:ESP32_DEVKIT`
- **Migrated to**: `skills/esp32/SKILL.md`
- **Content**:
  - GPIO pin capabilities (digital, analog, PWM, interrupts)
  - ADC1/ADC2 channels and WiFi conflicts
  - Communication interfaces (UART, I2C, SPI)
  - Strapping pins and boot constraints
  - Touch sensing, DAC, RTC GPIO
  - Pin selection best practices
  - Power specifications

#### Arduino Uno
- **Source**: `src/agent/skillsets.py:ARDUINO_UNO`
- **Migrated to**: `skills/arduino-uno/SKILL.md`
- **Content**:
  - ATmega328P specifications
  - Digital I/O and PWM pins
  - Analog input (10-bit ADC)
  - External interrupts (pins 2, 3)
  - Communication interfaces (UART, I2C, SPI)
  - Timer usage and constraints
  - Memory limitations (2KB SRAM, 32KB Flash)
  - Pin selection guidelines

### Frameworks

#### Arduino Framework for ESP32
- **Source**: `src/agent/graph.py` (Arduino code generation logic)
- **Migrated to**: `skills/esp32/arduino/SKILL.md`
- **Content**:
  - Arduino API for ESP32
  - ESP32-specific libraries (WiFi, BluetoothSerial, WebServer, Preferences)
  - LEDC peripheral for PWM
  - ESP32Servo library usage
  - Multi-core task creation
  - Touch sensing, DAC output
  - Differences from standard Arduino

#### Arduino Framework for Arduino Uno
- **Source**: Standard Arduino knowledge
- **Migrated to**: `skills/arduino-uno/arduino/SKILL.md`
- **Content**:
  - Core Arduino API
  - Standard libraries (Servo, Wire, SPI, SoftwareSerial, EEPROM)
  - Timer usage patterns
  - Memory management for 2KB SRAM
  - Non-blocking code patterns
  - Interrupt handling

### Components

#### Servo Motor (ESP32 + Arduino)
- **Source**: `templates/arduino/servo_template.ino`, `src/agent/graph.py`
- **Migrated to**: `skills/esp32/arduino/components/servo/SKILL.md`
- **Content**:
  - ESP32Servo library usage
  - LEDC-based PWM configuration
  - Pin selection (avoid strapping pins)
  - Code templates (sweep, position, interactive, multiple servos)
  - Pulse width calibration
  - Power considerations
  - Wiring diagrams

#### Servo Motor (Arduino Uno + Arduino)
- **Source**: `templates/arduino/servo_template.ino`
- **Migrated to**: `skills/arduino-uno/arduino/components/servo/SKILL.md`
- **Content**:
  - Standard Servo.h library
  - Timer1 usage and PWM conflicts (pins 9, 10)
  - Pin selection (prefer 9, 10 for 490Hz)
  - Code templates (sweep, position, potentiometer control, button control)
  - Multiple servo support
  - Power and current considerations
  - Wiring diagrams

### Framework Extensions

#### ESP-IDF Framework for ESP32
- **New**: `skills/esp32/esp-idf/SKILL.md`
- **Content**:
  - FreeRTOS task-based architecture
  - Component architecture
  - GPIO, LEDC, UART, I2C drivers
  - WiFi and event loop
  - NVS (non-volatile storage)
  - Build system (CMake)
  - Logging and error handling

### Utilities

#### Platform Detection Script
- **New**: `resources/scripts/detect_platform.py`
- **Purpose**: Auto-detect platform and framework from project files
- **Supports**:
  - platformio.ini parsing
  - ESP-IDF CMakeLists.txt detection
  - Arduino .ino file inference
  - .claude/project.json configuration

### Templates

#### ESP32 Arduino Servo
- **Source**: `templates/arduino/servo_template.ino` (adapted)
- **Migrated to**: `skills/esp32/arduino/components/servo/resources/templates/basic_sweep.ino`

#### Arduino Uno Servo
- **Source**: `templates/arduino/servo_template.ino`
- **Migrated to**: `skills/arduino-uno/arduino/components/servo/resources/templates/basic_sweep.ino`

## Architecture Changes

### From Monolithic to Hierarchical

**Before** (esp-arduino-agent):
```
src/agent/
├── skillsets.py           # All platforms in one file
├── graph.py               # Code generation logic mixed with platform knowledge
└── templates/arduino/     # Flat template directory
```

**After** (hardware-skills):
```
skills/
├── esp32/
│   ├── SKILL.md          # Platform knowledge
│   ├── arduino/
│   │   ├── SKILL.md      # Framework knowledge
│   │   └── components/
│   │       └── servo/
│   │           ├── SKILL.md        # Component generation
│   │           └── resources/      # Templates
│   └── esp-idf/
│       └── SKILL.md
└── arduino-uno/
    ├── SKILL.md
    └── arduino/
        └── components/
            └── servo/
```

### Benefits

1. **Separation of Concerns**:
   - Hardware specs (platform) separated from framework API
   - Code generation separated from knowledge base

2. **Framework Independence**:
   - Same platform (ESP32) supports multiple frameworks (Arduino, ESP-IDF)
   - Easy to add new frameworks without duplicating platform knowledge

3. **Component Specialization**:
   - Each component skill is specialized for platform + framework combination
   - No conditional logic based on platform/framework

4. **Extensibility**:
   - Add new platform: Create `skills/new-platform/SKILL.md`
   - Add new framework: Create `skills/platform/new-framework/SKILL.md`
   - Add new component: Create `skills/platform/framework/components/new-component/`

5. **Reusability**:
   - Skills can be used by any agent, not just esp-arduino-agent
   - Follows Claude Code Skills official specification
   - Compatible with other Claude Code plugins

## Breaking Changes

None - this is a new plugin, not a modification of esp-arduino-agent.

## Migration Notes

### For esp-arduino-agent Users

The original `esp-arduino-agent` project can continue to work independently, or it can be updated to use these skills:

**Option 1: Use as plugin**
```python
# In graph.py, load skills instead of skillsets
from claude_code_skills import load_skill

platform_skill = load_skill("esp32")
framework_skill = load_skill("esp32/arduino")
component_skill = load_skill("esp32/arduino/components/servo")
```

**Option 2: Keep both**
- `esp-arduino-agent`: Interactive LangGraph agent for project generation
- `hardware-skills`: Reusable knowledge base for all agents

### For New Users

Simply install the plugin and skills are automatically available in Claude Code.

## Future Plans

### Platforms
- ESP32-S3 (USB OTG, larger SRAM)
- ESP32-C3 (RISC-V, single core)
- ESP8266 (older WiFi chip)
- Arduino Mega (more pins, more memory)
- Arduino Nano (compact form factor)
- STM32F4 (ARM Cortex-M4)
- Raspberry Pi Pico (RP2040, dual-core ARM)

### Frameworks
- STM32Cube for STM32
- Pico SDK for RP2040
- PlatformIO (meta-framework)
- Zephyr RTOS

### Components
- WiFi (ESP32)
- Bluetooth (ESP32)
- LCD displays (I2C, parallel)
- OLED displays (SSD1306)
- DHT sensors (temperature/humidity)
- Ultrasonic sensors (HC-SR04)
- Stepper motors
- DC motors with H-bridge
- Relays and switching
- Button handling with debounce
- Rotary encoders
- IMU sensors (accelerometer/gyroscope)

### Enhancements
- Code validation scripts
- Automatic wiring diagram generation
- Pin conflict detection
- Power consumption estimation
- Component compatibility matrix

## Contributors

Initial migration and architecture by Claude Code based on esp-arduino-agent codebase.

## License

MIT License - see LICENSE file in parent directory
