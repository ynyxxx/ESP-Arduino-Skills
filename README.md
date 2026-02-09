# Hardware Skills for AI Agents

A comprehensive collection of skills for embedded hardware development across multiple platforms and frameworks.

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

Skills are organized in a two-layer structure:

```
skills/
├── platforms/                       # Layer 1: Platform reference (user-invocable: false)
│   ├── esp32/
│   │   └── SKILL.md                # ESP32 hardware specifications
│   └── arduino-uno/
│       └── SKILL.md                # Arduino Uno hardware specifications
│
└── [platform-framework-component]/ # Layer 2: Implementation skills
    ├── SKILL.md                    # Merged framework + component knowledge
    └── resources/
        └── templates/              # Code templates
```

**Example:**
```
skills/
├── platforms/esp32/SKILL.md        # ESP32 hardware reference
├── esp32-arduino-servo/            # ESP32 + Arduino + Servo implementation
│   ├── SKILL.md
│   └── resources/templates/
└── esp32-idf/                      # ESP32 + ESP-IDF (pure framework)
    └── SKILL.md
```

### Skill Hierarchy

**Layer 1: Platform Skills** (e.g., `platforms/esp32`, `platforms/arduino-uno`):
- Hardware specifications
- GPIO pin capabilities
- Communication interfaces
- Power requirements
- Pin selection best practices
- `user-invocable: false` (automatically loaded as reference)

**Layer 2: Implementation Skills** (e.g., `esp32-arduino-servo`, `arduino-uno-servo`):
- Merged framework + component knowledge
- Code generation for specific use cases
- Pin validation and selection
- Wiring diagrams
- Platform-specific implementation with framework API
- Can be invoked by user or Claude
- References platform skill for hardware details

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
1. skills/platforms/esp32/SKILL.md          # Platform reference
2. skills/esp32-arduino-servo/SKILL.md      # Implementation skill
```

### Manual Invocation

Invoke implementation skills directly:

```bash
# Generate ESP32 servo code (Arduino framework)
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
skills/
├── platforms/esp32/
│   └── SKILL.md                       # ESP32 hardware (GPIO, ADC, PWM, etc.)
└── esp32-arduino-servo/
    ├── SKILL.md                       # Merged: Arduino framework + Servo component
    └── resources/
        └── templates/
            └── basic_sweep.ino
```

**How it works**:
1. User asks: "Add servo to my ESP32 project"
2. Claude detects platform from context or asks
3. Auto-loads `platforms/esp32/SKILL.md` for hardware knowledge (pin constraints)
4. Loads `esp32-arduino-servo/SKILL.md` for framework API + servo implementation
5. Generates complete, validated code with proper pin selection

**Benefits**:
- Simpler directory structure (2 layers vs 3)
- Faster skill discovery
- Clearer naming (`esp32-arduino-servo` is self-documenting)
- Less cognitive overhead for users

### Arduino Uno + Arduino + Servo

```
skills/
├── platforms/arduino-uno/
│   └── SKILL.md                       # Arduino Uno hardware
└── arduino-uno-servo/
    ├── SKILL.md                       # Merged: Arduino framework + Servo component
    └── resources/
        └── templates/
            └── basic_sweep.ino
```

## Adding New Platforms

To add a new platform (e.g., STM32):

1. **Create platform reference skill**:
   ```
   skills/platforms/stm32f4/
   └── SKILL.md  # Hardware specs, GPIO, peripherals
   ```
   - Set `user-invocable: false`
   - Include complete hardware specifications

2. **Create implementation skills** (one per framework-component combo):
   ```
   skills/stm32f4-arduino-servo/
   ├── SKILL.md                    # Merged framework + servo knowledge
   └── resources/
       └── templates/
           └── basic_sweep.ino
   ```

## Adding New Components

To add a new component (e.g., WiFi):

1. **For each platform + framework combination**, create:
   ```
   skills/esp32-arduino-wifi/
   ├── SKILL.md                    # Merged framework + WiFi knowledge
   └── resources/
       └── templates/
           └── wifi_example.ino
   ```

2. **Implementation skill should**:
   - Reference parent platform skill for hardware details (cross-reference)
   - Include ~20% essential framework knowledge (setup/loop, Serial, etc.)
   - Include 100% component implementation
   - Generate complete, working code
   - Include wiring diagrams
   - Handle errors and edge cases

3. **Naming convention**: `{platform}-{framework}-{component}`
   - Example: `esp32-arduino-servo`
   - Example: `arduino-uno-arduino-lcd`

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

### Platform Skills (Layer 1)
- Focus on hardware specs (voltage, pins, peripherals)
- Include pin selection guidelines
- List constraints and gotchas
- Platform-agnostic (no framework-specific code)
- Set `user-invocable: false` (reference only)

### Implementation Skills (Layer 2)
- Merge framework basics (~20%) + component implementation (100%)
- Generate complete, working code
- Validate pin selection against platform skill
- Include wiring instructions
- Provide multiple usage examples
- Handle edge cases (power, timing, conflicts)
- Use clear cross-references to platform skill (e.g., "See `esp32` platform skill for complete GPIO specifications")
- Avoid duplicating platform hardware specifications

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
