# Hardware Skills for Claude Code

A comprehensive collection of Claude Code skills for embedded hardware development across multiple platforms and frameworks.

## Overview

This plugin provides hardware platform knowledge, framework-specific guidance, and component code generation skills for embedded development. Skills are organized hierarchically by platform → framework → component.

## Supported Platforms

### ESP32
- **Hardware**: ESP32 DevKit V1 (ESP32-WROOM-32)
- **Frameworks**: Arduino, ESP-IDF
- **Features**: WiFi, Bluetooth, dual-core, extensive GPIO

### Arduino Uno
- **Hardware**: Arduino Uno R3 (ATmega328P)
- **Frameworks**: Arduino
- **Features**: Standard Arduino platform, 5V logic

## Supported Components

### Actuators
- **Servo Motors**: Position control, continuous rotation, multi-servo

## Architecture

Skills are organized in a nested structure:

```
platform/
├── SKILL.md                    # Hardware specifications (pins, specs, constraints)
└── framework/
    ├── SKILL.md                # Framework-specific API and patterns
    └── components/
        └── component-name/
            ├── SKILL.md        # Component code generation
            └── resources/
                └── templates/  # Code templates
```

### Skill Hierarchy

**Platform Skills** (e.g., `esp32`, `arduino-uno`):
- Hardware specifications
- GPIO pin capabilities
- Communication interfaces
- Power requirements
- Pin selection best practices
- `user-invocable: false` (reference only)

**Framework Skills** (e.g., `esp32/arduino`, `arduino-uno/arduino`):
- Framework API documentation
- Library ecosystem
- Build configuration
- Framework-specific patterns
- `user-invocable: false` (reference only)

**Component Skills** (e.g., `esp32/arduino/components/servo`):
- Code generation for specific components
- Pin validation and selection
- Wiring diagrams
- Platform + framework specific implementation
- Can be invoked by user or Claude

## Installation

### As Claude Code Plugin

1. **Copy to plugins directory**:
   ```bash
   cp -r hardware-skills ~/.claude/plugins/hardware-skills
   ```

2. **Or use as project-local plugin**:
   ```bash
   cp -r hardware-skills .claude/plugins/hardware-skills
   ```

### Skills are Auto-Discovered

Claude Code automatically discovers skills in:
- `~/.claude/plugins/*/skills/` (personal plugins)
- `.claude/plugins/*/skills/` (project plugins)

## Usage

### Automatic Invocation

Claude automatically loads relevant skills based on context:

```
User: "Create servo control code for ESP32"

Claude loads (automatically):
1. skills/esp32/SKILL.md
2. skills/esp32/arduino/SKILL.md
3. skills/esp32/arduino/components/servo/SKILL.md
```

### Manual Invocation

Invoke component skills directly:

```bash
# Generate ESP32 servo code
/esp32-arduino-servo

# Generate Arduino Uno servo code
/arduino-uno-servo
```

### Project Configuration

Create `.claude/project.json` to specify platform and framework:

```json
{
  "platform": "esp32",
  "framework": "arduino"
}
```

Skills can detect this configuration and auto-adapt.

## Skill Organization Examples

### ESP32 + Arduino + Servo

```
skills/esp32/
├── SKILL.md                           # ESP32 hardware (GPIO, ADC, PWM, etc.)
└── arduino/
    ├── SKILL.md                       # Arduino framework for ESP32
    └── components/
        └── servo/
            ├── SKILL.md               # Servo code generation
            └── resources/
                └── templates/
                    └── basic_sweep.ino
```

**How it works**:
1. User asks: "Add servo to my ESP32 project"
2. Claude detects platform from context or asks
3. Loads `esp32/SKILL.md` for hardware knowledge (pin constraints)
4. Loads `esp32/arduino/SKILL.md` for framework API
5. Loads `esp32/arduino/components/servo/SKILL.md` to generate code
6. Generates complete, validated code with proper pin selection

### Arduino Uno + Arduino + Servo

```
skills/arduino-uno/
├── SKILL.md                           # Arduino Uno hardware
└── arduino/
    ├── SKILL.md                       # Standard Arduino framework
    └── components/
        └── servo/
            ├── SKILL.md               # Servo for Uno
            └── resources/
                └── templates/
                    └── basic_sweep.ino
```

## Adding New Platforms

To add a new platform (e.g., STM32):

1. **Create platform skill**:
   ```
   skills/stm32f4/
   └── SKILL.md  # Hardware specs, GPIO, peripherals
   ```

2. **Add framework support**:
   ```
   skills/stm32f4/
   ├── SKILL.md
   └── arduino/
       └── SKILL.md  # Arduino for STM32
   ```

3. **Add component implementations**:
   ```
   skills/stm32f4/arduino/components/
   └── servo/
       ├── SKILL.md
       └── resources/
   ```

## Adding New Components

To add a new component (e.g., WiFi):

1. **For each platform + framework combination**, create:
   ```
   skills/esp32/arduino/components/wifi/
   ├── SKILL.md
   └── resources/
       └── templates/
           └── wifi_example.ino
   ```

2. **Component skill should**:
   - Reference parent platform skill for pin selection
   - Reference parent framework skill for API usage
   - Generate complete, working code
   - Include wiring diagrams
   - Handle errors and edge cases

## Utility Scripts

### Detect Platform

```bash
python resources/scripts/detect_platform.py .
```

Returns JSON with detected platform and framework:
```json
{
  "platform": "esp32",
  "framework": "arduino",
  "source": "platformio.ini"
}
```

## Best Practices

### Platform Skills
- Focus on hardware specs (voltage, pins, peripherals)
- Include pin selection guidelines
- List constraints and gotchas
- Platform-agnostic (no framework-specific code)

### Framework Skills
- Focus on API and libraries
- Include build configuration
- Show common patterns
- Reference platform skill for hardware details

### Component Skills
- Generate complete, working code
- Validate pin selection against platform
- Include wiring instructions
- Provide multiple usage examples
- Handle edge cases (power, timing, conflicts)

## Skill Naming Convention

- **Platform**: `platform-name` (e.g., `esp32`, `arduino-uno`)
- **Framework**: `platform-framework` (e.g., `esp32-arduino`)
- **Component**: `platform-framework-component` (e.g., `esp32-arduino-servo`)

This avoids naming conflicts across the skill ecosystem.

## Development

### Testing Skills

1. Copy to Claude Code plugins directory
2. Test auto-invocation:
   ```
   Ask Claude: "I need servo control for ESP32"
   ```
3. Test manual invocation:
   ```
   /esp32-arduino-servo
   ```
4. Verify generated code compiles

### Debugging

- Check skill is discovered: Ask Claude "What skills are available?"
- Verify YAML frontmatter is valid
- Check `user-invocable` setting
- Review `description` field (used for auto-matching)

## Roadmap

### Planned Platforms
- [x] ESP32
- [x] Arduino Uno
- [ ] ESP32-S3
- [ ] ESP32-C3
- [ ] Arduino Mega
- [ ] Arduino Nano
- [ ] STM32F4
- [ ] Raspberry Pi Pico (RP2040)

### Planned Frameworks
- [x] Arduino Framework
- [ ] ESP-IDF
- [ ] STM32Cube
- [ ] Pico SDK
- [ ] PlatformIO (meta-framework)

### Planned Components
- [x] Servo Motors
- [ ] WiFi (ESP32)
- [ ] Bluetooth (ESP32)
- [ ] LCD Displays (I2C, SPI)
- [ ] OLED Displays
- [ ] DHT Sensors (Temperature/Humidity)
- [ ] Ultrasonic Sensors
- [ ] Stepper Motors
- [ ] DC Motors
- [ ] Relays
- [ ] Buttons/Interrupts

## Contributing

To contribute new platforms, frameworks, or components:

1. Follow the existing directory structure
2. Use the SKILL.md format (YAML frontmatter + Markdown)
3. Include code templates in `resources/templates/`
4. Add wiring diagrams and examples
5. Test with Claude Code before submitting

## License

MIT License - see parent project LICENSE file

## Resources

- [Claude Code Skills Documentation](https://code.claude.com/docs/en/skills)
- [Agent Skills Specification](https://agentskills.io)
- [Arduino Reference](https://www.arduino.cc/reference/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
