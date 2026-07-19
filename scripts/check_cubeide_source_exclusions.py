#!/usr/bin/env python3
"""Validate STM32CubeIDE source exclusions for wrapper translation units.

Several productive *_patched.c files include their base .c implementation inside
one translation unit. STM32CubeIDE scans Core/Src automatically, so the base
files must be excluded from Debug and Release or the linker sees every public
symbol twice.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_FILE = Path(".cproject")
REQUIRED_EXCLUSIONS = {
    "Src/motor_control.c",
    "Src/safety_system.c",
    "Src/sensor_manager.c",
    "Src/steering_centering.c",
}
REQUIRED_CONFIGS = {"Debug", "Release"}


def fail(message: str) -> None:
    print(f"CubeIDE source check FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    if not PROJECT_FILE.is_file():
        fail(f"missing {PROJECT_FILE}")

    try:
        root = ET.parse(PROJECT_FILE).getroot()
    except ET.ParseError as exc:
        fail(f"invalid XML in {PROJECT_FILE}: {exc}")

    checked: set[str] = set()
    for configuration in root.findall(".//configuration"):
        name = configuration.get("name")
        if name not in REQUIRED_CONFIGS:
            continue

        core_entries = [
            entry
            for entry in configuration.findall("./sourceEntries/entry")
            if entry.get("name") == "Core"
        ]
        if len(core_entries) != 1:
            fail(f"{name}: expected one Core source entry, found {len(core_entries)}")

        exclusions = {
            item
            for item in core_entries[0].get("excluding", "").split("|")
            if item
        }
        missing = sorted(REQUIRED_EXCLUSIONS - exclusions)
        if missing:
            fail(f"{name}: missing exclusions: {', '.join(missing)}")
        checked.add(name)

    missing_configs = sorted(REQUIRED_CONFIGS - checked)
    if missing_configs:
        fail(f"missing configurations: {', '.join(missing_configs)}")

    print(
        "CubeIDE source check OK: Debug/Release compile only the productive "
        "*_patched.c wrappers for motor, safety, sensors and steering centering."
    )


if __name__ == "__main__":
    main()
