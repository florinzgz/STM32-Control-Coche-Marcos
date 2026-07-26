#!/usr/bin/env python3
"""Validate the EPS symbol that the production Makefile actually compiles."""

from pathlib import Path
import re


def strip_comments_and_literals(source: str) -> str:
    token = re.compile(
        r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        re.DOTALL,
    )

    def blank(match: re.Match[str]) -> str:
        return ''.join('\n' if ch == '\n' else ' ' for ch in match.group(0))

    return token.sub(blank, source)


def braced_block(source: str, opening: int, label: str) -> str:
    depth = 0
    for pos in range(opening, len(source)):
        char = source[pos]
        if char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return source[opening + 1:pos]
    raise AssertionError(f'{label}: closing brace not found')


def function_body(source: str, signature: str, label: str) -> str:
    matches = list(re.finditer(signature, source, re.MULTILINE))
    assert len(matches) == 1, (
        f'{label}: expected one effective definition, found {len(matches)}')
    opening = source.find('{', matches[0].end())
    assert opening >= 0, f'{label}: opening brace not found'
    return braced_block(source, opening, label)


def require(pattern: str, source: str, label: str) -> re.Match[str]:
    match = re.search(pattern, source, re.MULTILINE | re.DOTALL)
    assert match is not None, f'{label}: required executable construct missing'
    return match


makefile = Path('Makefile').read_text(encoding='utf-8')
assert re.search(
    r'(?m)^\s*\$\(CORE_SRC\)/motor_control_patched\.c\s*\\?\s*$',
    makefile,
), 'production Makefile does not compile motor_control_patched.c'
assert not re.search(
    r'(?m)^\s*\$\(CORE_SRC\)/motor_control\.c\s*\\?\s*$',
    makefile,
), 'motor_control.c unexpectedly compiled as a standalone translation unit'

raw_wrapper = Path('Core/Src/motor_control_patched.c').read_text(encoding='utf-8')
assert re.search(
    r'#define\s+Steering_ControlLoop\s+Steering_ControlLoop_Base\s*\n'
    r'#include\s+"motor_control\.c"\s*\n'
    r'#undef\s+Steering_ControlLoop',
    raw_wrapper,
), 'base EPS symbol is not renamed before textual inclusion'

wrapper = strip_comments_and_literals(raw_wrapper)
body = function_body(
    wrapper,
    r'\bvoid\s+Steering_ControlLoop\s*\(\s*void\s*\)',
    'effective Steering_ControlLoop()',
)

power_gate = require(
    r'\bif\s*\(\s*!\s*Safety_IsPowerReady\s*\(\s*\)\s*\|\|\s*'
    r'HAL_GPIO_ReadPin\s*\(\s*GPIOC\s*,\s*PIN_RELAY_STEER_PWR\s*\)\s*'
    r'!=\s*GPIO_PIN_SET\s*\)',
    body,
    'effective EPS power/PC12 gate',
)
power_block_open = body.find('{', power_gate.end())
assert power_block_open >= 0, 'effective EPS power gate block missing'
power_block = braced_block(body, power_block_open, 'effective EPS power gate')
require(r'\bSteering_Neutralize\s*\(\s*\)\s*;', power_block,
        'power gate neutralization')
require(r'\breturn\s*;', power_block, 'power gate return')

params = require(r'\bEPS_Params_Get\s*\(\s*\)', body, 'EPS parameters')
assert power_gate.start() < params.start(), (
    'power/PC12 gate must execute before EPS calculations')

policy = require(
    r'\bEpsOutput_Resolve\s*\(\s*pwm_pct\s*,\s*'
    r'p->coast_band_pct\s*,\s*p->min_drive_pct\s*\)',
    body,
    'coast-first output policy',
)
coast = require(r'\bif\s*\(\s*output\.coast\s*\)', body,
                'coast decision')
resolved_pwm = require(
    r'\bpwm_pct\s*=\s*output\.pwm_pct\s*;', body,
    'resolved minimum-drive output',
)
pwm_conversion = require(
    r'\bint16_t\s+pwm_raw\s*=\s*\(int16_t\)', body,
    'PWM conversion',
)
assert policy.start() < coast.start() < resolved_pwm.start() < pwm_conversion.start(), (
    'effective EPS order must be resolve -> coast -> resolved PWM -> counts')

assert not re.search(r'abs_pct\s*>\s*0\.01f', body), (
    'legacy minimum-drive-before-coast block still exists in effective wrapper')
assert not re.search(
    r'pwm_pct\s*=\s*\([^;]*\?\s*p->min_drive_pct', body), (
    'effective wrapper still promotes PWM directly before the coast policy')

print('Effective EPS wrapper: production source, power gate and coast-first order OK')
