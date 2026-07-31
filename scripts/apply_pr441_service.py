#!/usr/bin/env python3
"""Apply the audited PR441 service changes through deterministic anchors.

Temporary integration helper.  The final productive commit removes this file.
"""

from __future__ import annotations

import base64
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATCH = Path('/tmp/pr441-service.patch')


def run(*args: str) -> None:
    subprocess.run(args, cwd=ROOT, check=True)


def decode_parts() -> None:
    alphabet = set('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=')
    with PATCH.open('wb') as output:
        for part in sorted((ROOT / '.github/pr441_patch').glob('part*.b64')):
            raw = part.read_text(encoding='ascii')
            clean = ''.join(ch for ch in raw if ch in alphabet)
            output.write(base64.b64decode(clean, validate=False))
    if PATCH.stat().st_size < 1000:
        raise SystemExit('staged service patch is unexpectedly small')


def integrate_motor_wrapper() -> None:
    path = ROOT / 'Core/Src/motor_control_patched.c'
    text = path.read_text(encoding='utf-8')

    include_anchor = '#include "eps_assist_policy.h"\n'
    helper_marker = 'PR_MotorPedalCalServiceActive'
    helper = '''#include "can_handler.h"

#ifdef HOST_TEST
extern bool CAN_PedalCalServiceActive(void) __attribute__((weak));
static bool PR_MotorPedalCalServiceActive(void)
{
    return CAN_PedalCalServiceActive != 0 &&
           CAN_PedalCalServiceActive();
}
#else
static bool PR_MotorPedalCalServiceActive(void)
{
    return CAN_PedalCalServiceActive();
}
#endif
'''
    if helper_marker not in text:
        if text.count(include_anchor) != 1:
            raise SystemExit('motor wrapper include anchor missing or ambiguous')
        text = text.replace(include_anchor, include_anchor + helper, 1)

    function_anchor = 'void Traction_Update(void)\n{\n'
    lock_marker = 'highest-priority physical motion inhibit'
    lock = '''    /* Pedal-calibration service lock is an independent,
     * highest-priority physical motion inhibit.  Do not evaluate ramps,
     * tank turn, Ackermann, ABS/TCS or base demand while it owns the vehicle. */
    if (PR_MotorPedalCalServiceActive()) {
        (void)Traction_CalibrationLock();
        coast_all();
        zero_output_telemetry();
        Traction_UpdateMotionInhibit(0.0f, 0U);
        return;
    }

'''
    if lock_marker not in text:
        if text.count(function_anchor) != 1:
            raise SystemExit('effective Traction_Update anchor missing or ambiguous')
        text = text.replace(function_anchor, function_anchor + lock, 1)

    if text.count(helper_marker) < 3:
        raise SystemExit('service helper was not integrated completely')
    if text.count(lock_marker) != 1:
        raise SystemExit('service lock insertion is not unique')

    path.write_text(text, encoding='utf-8')


def main() -> None:
    decode_parts()
    run('sha256sum', str(PATCH))
    run('git', 'apply', '--3way', '--exclude=Core/Src/motor_control_patched.c', str(PATCH))
    integrate_motor_wrapper()
    run('git', 'diff', '--check')


if __name__ == '__main__':
    main()
