# Quick Start Guide

## Installation

### Option 1: Personal Plugin (Recommended)
Install for all your projects:

```bash
# Copy to Claude Code personal plugins directory
cp -r hardware-skills ~/.claude/plugins/hardware-skills
```

### Option 2: Project Plugin
Install for current project only:

```bash
# Copy to project's Claude plugins directory
cp -r hardware-skills .claude/plugins/hardware-skills
```

## Verify Installation

Ask Claude Code:

```
What skills are available?
```

You should see skills like:
- `esp32` (platform reference - in platforms/)
- `arduino-uno` (platform reference - in platforms/)
- `esp32-arduino-servo` (implementation skill)
- `arduino-uno-servo` (implementation skill)
- `esp32-idf` (implementation skill - pure framework)

## Basic Usage

### Example 1: Generate Servo Code for ESP32

**User:**
```
Generate servo control code for ESP32
```

**Claude will:**
1. Auto-load `platforms/esp32/SKILL.md` for hardware specs
2. Auto-load `esp32-arduino-servo/SKILL.md` for framework API + servo implementation
3. Generate complete code with proper pin selection

### Example 2: Manual Invocation

```
/esp32-arduino-servo
```

Directly invokes the servo component skill for ESP32+Arduino.

### Example 3: With Project Configuration

Create `.claude/project.json`:

```json
{
  "platform": "esp32",
  "framework": "arduino"
}
```

Now when you ask for components, Claude automatically knows the context:

```
User: Add servo control

Claude: (auto-detects ESP32 + Arduino from config, generates appropriate code)
```

## Skills Overview

### Layer 1: Platform Skills (Hardware Reference)

Located in `skills/platforms/`:
- **platforms/esp32**: ESP32 GPIO, ADC, PWM, WiFi/BT capabilities
- **platforms/arduino-uno**: Arduino Uno pins, timers, memory constraints

These provide hardware specifications and pin selection guidance.
Set to `user-invocable: false` - automatically loaded by Claude as reference.

### Layer 2: Implementation Skills (Framework + Component)

Located in `skills/`:
- **esp32-arduino-servo**: Arduino framework + Servo for ESP32
- **arduino-uno-servo**: Arduino framework + Servo for Arduino Uno
- **esp32-idf**: ESP-IDF native framework (pure framework, no component yet)

These merge framework API knowledge with component implementation.
Generate actual working code with wiring diagrams.

## Common Workflows

### Create New ESP32 Servo Project

```
User: Create an ESP32 project that controls a servo motor

Claude:
1. Generates platformio.ini or .ino file
2. Includes ESP32Servo library
3. Selects safe GPIO pin (e.g., 18)
4. Provides complete code
5. Shows wiring diagram
6. Lists power requirements
```

### Add Component to Existing Project

```
User: I have an Arduino Uno. Add a servo that sweeps 180 degrees.

Claude:
1. Detects Arduino Uno platform
2. Generates standard Servo.h code
3. Uses pin 9 (best for servos on Uno)
4. Includes sweep code
5. Notes Timer1 usage
```

### Check Pin Compatibility

```
User: Can I use GPIO 34 for a servo on ESP32?

Claude:
(Checks platforms/esp32 platform skill)
No, GPIO 34 is input-only. Recommended pins for servo: 16, 17, 18, 19, 21-23
```

## Advanced Usage

### Platform Detection

Skills can auto-detect platform from:

```bash
# Run detection script
python hardware-skills/resources/scripts/detect_platform.py .
```

Returns JSON with platform and framework info.

### Custom Pin Assignment

```
User: Use GPIO 25 for the servo instead

Claude: (Validates GPIO 25 is suitable, generates code with GPIO 25)
```

### Multiple Components

```
User: Add WiFi connection and servo control to ESP32

Claude:
(Loads esp32-arduino-wifi and esp32-arduino-servo skills, generates integrated code)
```

## Troubleshooting

### Skills Not Found

1. Check installation path:
   ```bash
   ls ~/.claude/plugins/hardware-skills/skills/
   ```

2. Verify SKILL.md files exist:
   ```bash
   find ~/.claude/plugins/hardware-skills -name "SKILL.md"
   ```

### Wrong Code Generated

Check if platform/framework is correctly detected:

```
User: What platform am I using?

Claude: (should detect from project files or .claude/project.json)
```

Explicitly specify if needed:

```
User: Generate servo code for ESP32 using Arduino framework
```

### Skill Not Invocable

Skills with `user-invocable: false` can't be called with `/skill-name`.

These are reference skills (platform, framework) that Claude loads automatically.

Only component skills can be manually invoked.

## Next Steps

1. **Explore existing skills**: See `skills/` directory
2. **Read platform guides**: Check platform SKILL.md files
3. **Try examples**: Use the templates in `resources/templates/`
4. **Extend**: Add new components following the same structure

## Getting Help

- Full documentation: `README.md`
- Architecture details: `CHANGELOG.md`
- Component examples: `skills/*/components/*/SKILL.md`

Happy building! 🚀
