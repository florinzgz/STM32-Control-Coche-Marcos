#!/usr/bin/env python3
"""Update any remaining test calls using the old rear-4x2 resolver signature."""
from pathlib import Path
import re

path = Path("Core/Src/test_traction_output_policy.c")
text = path.read_text(encoding="utf-8")
pattern = re.compile(
    r"TractionOutput_Resolve4x2Rear\(mode, direction, pwm, scales,\n"
    r"(?P<indent>\s*)4249U, (?P<out>NULL|&p)\)"
)
text, count = pattern.subn(
    lambda m: "TractionOutput_Resolve4x2Rear(mode, direction, pwm, "
              "3.0f, 2.0f,\n" + m.group("indent") +
              "scales, 4249U, " + m.group("out") + ")",
    text,
)
if count != 2:
    raise RuntimeError(f"expected two remaining old-signature calls, found {count}")
path.write_text(text, encoding="utf-8")
print("remaining 4x2 test calls updated")
