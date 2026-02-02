#!/usr/bin/env python3
"""
Detect platform and framework from project configuration.

Usage:
    python detect_platform.py [project_dir]

Returns:
    JSON with platform and framework information
"""

import json
import os
import sys
from pathlib import Path


def detect_platformio(project_dir):
    """Detect platform/framework from platformio.ini"""
    ini_file = project_dir / "platformio.ini"
    if not ini_file.exists():
        return None

    content = ini_file.read_text()
    platform = None
    framework = None

    for line in content.split('\n'):
        line = line.strip()
        if line.startswith('platform ='):
            platform_val = line.split('=')[1].strip()
            if 'esp32' in platform_val or 'espressif32' in platform_val:
                platform = 'esp32'
            elif 'atmelavr' in platform_val:
                platform = 'arduino-uno'

        if line.startswith('framework ='):
            framework_val = line.split('=')[1].strip()
            if framework_val == 'arduino':
                framework = 'arduino'
            elif framework_val == 'espidf':
                framework = 'esp-idf'

    if platform and framework:
        return {'platform': platform, 'framework': framework, 'source': 'platformio.ini'}
    return None


def detect_arduino_cli(project_dir):
    """Detect platform from Arduino CLI project"""
    # Check for .ino files
    ino_files = list(project_dir.glob('*.ino'))
    if not ino_files:
        return None

    # Simple heuristic: check for ESP32-specific includes
    for ino_file in ino_files:
        content = ino_file.read_text()
        if '#include <WiFi.h>' in content or '#include <ESP32Servo.h>' in content:
            return {'platform': 'esp32', 'framework': 'arduino', 'source': 'inference from .ino'}
        elif '#include <Servo.h>' in content:
            return {'platform': 'arduino-uno', 'framework': 'arduino', 'source': 'inference from .ino'}

    return {'platform': 'unknown', 'framework': 'arduino', 'source': 'default guess'}


def detect_esp_idf(project_dir):
    """Detect ESP-IDF project"""
    if (project_dir / "CMakeLists.txt").exists():
        cmake_content = (project_dir / "CMakeLists.txt").read_text()
        if 'idf_component_register' in cmake_content or 'ESP-IDF' in cmake_content:
            return {'platform': 'esp32', 'framework': 'esp-idf', 'source': 'CMakeLists.txt'}
    return None


def detect_project_config(project_dir):
    """Detect from .claude/project.json config file"""
    config_file = project_dir / ".claude" / "project.json"
    if config_file.exists():
        try:
            config = json.loads(config_file.read_text())
            if 'platform' in config and 'framework' in config:
                return {
                    'platform': config['platform'],
                    'framework': config['framework'],
                    'source': '.claude/project.json'
                }
        except:
            pass
    return None


def main():
    if len(sys.argv) > 1:
        project_dir = Path(sys.argv[1])
    else:
        project_dir = Path.cwd()

    # Try detection methods in priority order
    result = (
        detect_project_config(project_dir) or
        detect_platformio(project_dir) or
        detect_esp_idf(project_dir) or
        detect_arduino_cli(project_dir)
    )

    if result:
        print(json.dumps(result, indent=2))
        return 0
    else:
        print(json.dumps({'platform': 'unknown', 'framework': 'unknown', 'source': 'none'}, indent=2))
        return 1


if __name__ == '__main__':
    sys.exit(main())
