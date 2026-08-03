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
require(r'\bSteering_FullNeutralize\s*\(\s*\)\s*;', power_block,
        'power gate full neutralization')
require(r'\breturn\s*;', power_block, 'power gate return')

full_neutral_body = function_body(
    wrapper,
    r'\bstatic\s+void\s+Steering_FullNeutralize\s*\(\s*void\s*\)',
    'Steering_FullNeutralize()',
)
require(r'\bEpsAssist_Reset\s*\(', full_neutral_body,
        'full neutral intent reset')
require(r'\bs_eps_raw_reversal_cycles\s*=\s*0U\s*;',
        full_neutral_body, 'full neutral reversal reset')
require(r'\bSteering_Neutralize\s*\(\s*\)\s*;', full_neutral_body,
        'full neutral estimator reset')

coast_helper_body = function_body(
    wrapper,
    r'\bstatic\s+void\s+Steering_CoastPreserveEstimator\s*'
    r'\(\s*void\s*\)',
    'Steering_CoastPreserveEstimator()',
)
require(r'\bMotor_SetSigned\s*\(\s*&motor_steer\s*,\s*0\s*\)\s*;',
        coast_helper_body, 'normal coast physical output off')
assert 'Steering_Neutralize' not in coast_helper_body, (
    'normal coast must not reset the EPS estimator')
assert 'eps_omega_filt' not in coast_helper_body, (
    'normal coast must preserve filtered driver intent')

params = require(r'\bEPS_Params_Get\s*\(\s*\)', body, 'EPS parameters')
assert power_gate.start() < params.start(), (
    'power/PC12 gate must execute before EPS calculations')

raw_reversal = require(
    r'\bEpsAssist_UpdateRawReversal\s*\(', body,
    'two-cycle raw/EMA reversal confirmation',
)
damped_assist = require(
    r'\bEpsAssist_ApplyDamping\s*\(', body,
    'sign-preserving assist damping',
)
assist_policy = require(
    r'\bEpsAssist_Resolve\s*\([^;]*\bdamped_assist_tau\b', body,
    'driver-intent assist policy with damping',
)
direction_change = require(
    r'\bEpsAssist_DirectionChanged\s*\(', body,
    'latched intent direction change detector',
)
direction_coast = require(
    r'\bif\s*\(\s*intent_direction_changed\s*\|\|\s*'
    r'raw_reversal_confirmed\s*\)', body,
    'zero-torque reversal transition',
)
preserve_coast = require(
    r'\bSteering_CoastPreserveEstimator\s*\(\s*\)\s*;', body,
    'observer-preserving normal coast',
)
effective_coast = require(
    r'\bEpsAssist_EffectiveCoastBand\s*\(', body,
    'bounded driver-assist coast threshold',
)
effective_min = require(
    r'\bEpsAssist_EffectiveMinDrive\s*\(', body,
    'bounded driver-assist breakaway',
)
policy = require(
    r'\bEpsOutput_Resolve\s*\(\s*pwm_pct\s*,\s*'
    r'effective_coast_band\s*,\s*effective_min_drive\s*\)',
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
direction_block_open = body.find('{', direction_coast.end())
assert direction_block_open >= 0, 'direction coast block missing'
# The opening brace must belong to the direction-coast if() — verify that only
# whitespace (no other token) sits between the condition end and the brace.
direction_interstitial = body[direction_coast.end():direction_block_open]
import re as _re
assert not _re.search(r'\S', direction_interstitial), (
    'direction-coast opening brace must immediately follow the if() condition, '
    f'not be separated by non-whitespace: {direction_interstitial!r}'
)
direction_block = braced_block(body, direction_block_open,
                               'direction coast block')
require(r'\bSteering_CoastPreserveEstimator\s*\(\s*\)\s*;',
        direction_block, 'direction-change physical coast')
require(r'\breturn\s*;', direction_block, 'direction-change return')

assert raw_reversal.start() < damped_assist.start() < assist_policy.start(), (
    'raw reversal and damping must precede intent resolution')
assert assist_policy.start() < direction_change.start() < direction_coast.start(), (
    'intent direction change must be checked immediately after resolution')
assert direction_coast.start() < effective_coast.start(), (
    'direction uncertainty must coast before output-policy/PWM processing')
assert assist_policy.start() < effective_coast.start() < effective_min.start() < policy.start(), (
    'driver intent must be resolved before coast/breakaway output policy')
assert policy.start() < coast.start() < resolved_pwm.start() < pwm_conversion.start(), (
    'effective EPS order must be resolve -> coast -> resolved PWM -> counts')
assert preserve_coast.start() < pwm_conversion.start(), (
    'normal coast must preserve estimator before any PWM conversion')

assert not re.search(r'abs_pct\s*>\s*0\.01f', body), (
    'legacy minimum-drive-before-coast block still exists in effective wrapper')
assert not re.search(
    r'pwm_pct\s*=\s*\([^;]*\?\s*p->min_drive_pct', body), (
    'effective wrapper still promotes PWM directly before the coast policy')

print('Effective EPS wrapper: power gate, reversal coast, damped assist and observer-preserving coast OK')
