#!/usr/bin/env python3
"""Validate source selection for wrapper translation units.

Several productive ``*_patched.c`` files include their base ``.c`` implementation
inside one translation unit.  The base file must therefore remain available as
source code, but it must never be compiled as a second object alongside its
wrapper.  This guard validates STM32CubeIDE metadata, the repository Makefile,
the wrapper include contracts and any generated Debug/Release manifests that
happen to exist in the workspace.
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT_FILE = ROOT / ".cproject"
MAKEFILE = ROOT / "Makefile"
CORE_SRC = ROOT / "Core" / "Src"

WRAPPER_PAIRS = {
    "motor_control.c": "motor_control_patched.c",
    "safety_system.c": "safety_system_patched.c",
    "sensor_manager.c": "sensor_manager_patched.c",
    "steering_centering.c": "steering_centering_patched.c",
}
REQUIRED_EXCLUSIONS = {f"Src/{base}" for base in WRAPPER_PAIRS}
REQUIRED_CONFIGS = {"Debug", "Release"}
GENERATED_MANIFESTS = (
    ROOT / "Debug" / "Core" / "Src" / "subdir.mk",
    ROOT / "Release" / "Core" / "Src" / "subdir.mk",
    ROOT / "Debug" / "objects.list",
    ROOT / "Release" / "objects.list",
)


def fail(message: str) -> None:
    print(f"CubeIDE source check FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        fail(f"cannot read {path.relative_to(ROOT)}: {exc}")


def validate_project_exclusions() -> None:
    if not PROJECT_FILE.is_file():
        fail(f"missing {PROJECT_FILE.relative_to(ROOT)}")

    try:
        root = ET.parse(PROJECT_FILE).getroot()
    except (OSError, ET.ParseError) as exc:
        fail(f"invalid {PROJECT_FILE.relative_to(ROOT)}: {exc}")

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


def validate_wrapper_contracts() -> None:
    make_text = read_text(MAKEFILE)
    make_sources = set(re.findall(r"\$\(CORE_SRC\)/([A-Za-z0-9_]+\.c)", make_text))

    for base_name, wrapper_name in WRAPPER_PAIRS.items():
        base_path = CORE_SRC / base_name
        wrapper_path = CORE_SRC / wrapper_name
        if not base_path.is_file():
            fail(f"missing base implementation Core/Src/{base_name}")
        if not wrapper_path.is_file():
            fail(f"missing productive wrapper Core/Src/{wrapper_name}")
        if base_path.stat().st_size == 0:
            fail(f"base implementation Core/Src/{base_name} is empty")

        wrapper_text = read_text(wrapper_path)
        include_token = f'#include "{base_name}"'
        include_count = wrapper_text.count(include_token)
        if include_count != 1:
            fail(
                f"Core/Src/{wrapper_name}: expected exactly one {include_token}, "
                f"found {include_count}"
            )

        if wrapper_name not in make_sources:
            fail(f"Makefile does not compile productive wrapper {wrapper_name}")
        if base_name in make_sources:
            fail(
                f"Makefile compiles both {base_name} and {wrapper_name}; "
                "this would duplicate linker symbols"
            )


def validate_generated_manifests() -> None:
    """Reject stale local CubeIDE manifests when they are present.

    Generated Debug/Release files are normally untracked, so CI may not have
    them.  On a developer workspace this catches the exact failure mode where an
    old ``subdir.mk`` or ``objects.list`` still mentions both objects even after
    ``.cproject`` was corrected.
    """

    for manifest in GENERATED_MANIFESTS:
        if not manifest.is_file():
            continue
        text = read_text(manifest)
        for base_name, wrapper_name in WRAPPER_PAIRS.items():
            base_object = base_name.removesuffix(".c") + ".o"
            wrapper_object = wrapper_name.removesuffix(".c") + ".o"
            base_present = base_name in text or base_object in text
            wrapper_present = wrapper_name in text or wrapper_object in text
            if base_present and wrapper_present:
                fail(
                    f"stale {manifest.relative_to(ROOT)} contains both "
                    f"{base_name} and {wrapper_name}; clean/delete Debug and "
                    "Release, then regenerate the CubeIDE build"
                )


def main() -> None:
    validate_project_exclusions()
    validate_wrapper_contracts()
    validate_generated_manifests()
    print(
        "CubeIDE source check OK: base implementations are intact, wrappers "
        "include them exactly once, Makefile selects only wrappers, and no "
        "present Debug/Release manifest contains duplicate objects."
    )


if __name__ == "__main__":
    main()
