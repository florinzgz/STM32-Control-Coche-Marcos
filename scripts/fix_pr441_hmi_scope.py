#!/usr/bin/env python3
"""Fix the PR441 pedal-calibration HMI service-lock scope.

Temporary integration helper; removed by the committing workflow.
"""
from pathlib import Path

path = Path('esp32/src/screens/engineering_screen.cpp')
text = path.read_text(encoding='utf-8')
old = '        const bool service_locked = sess_fresh && sess_active;\n'
new = '''        const bool service_locked =
            pedalcal::sessionFreshness(pedalSessLastTs_, (unsigned long)millis())
                == pedalcal::Freshness::FRESH &&
            pedalcal::sessionActive(pedalSessState_);
'''
if text.count(old) != 1:
    raise SystemExit('SERVICE LOCK scope anchor missing or ambiguous')
path.write_text(text.replace(old, new, 1), encoding='utf-8')
