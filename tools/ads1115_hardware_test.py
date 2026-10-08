#!/usr/bin/env python3
"""Explicit hardware-in-the-loop test. Stop the ADC server before running."""
import argparse
import copy
import json
import time
from pathlib import Path

from peaceofmine_operator.ads1115 import ADS1115, DEFAULTS, INPUTS, RANGES, RATES, config_word


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--address', type=lambda s: int(s, 0), default=0x48)
    parser.add_argument('--report', type=Path, default=Path('/tmp/ads1115-hardware-results.json'))
    args = parser.parse_args()
    adc = ADS1115(args.bus, args.address)
    original = adc.registers()
    print('Original registers:', {k: f'0x{v:04X}' for k, v in original.items()}, flush=True)
    results = []
    c = copy.deepcopy(DEFAULTS)
    try:
        # Exhaust all distinct MUX/PGA/DR combinations and check each readback.
        for rate in RATES:
            for pga in range(6):
                c.update(rate=rate, pga=pga)
                adc.configure(c)
                for mux in range(8):
                    start = time.monotonic()
                    raw = adc.sample(c, mux)
                    actual = adc.read(1)
                    expected = config_word(c, mux)
                    assert actual & 0x7fff == expected, (hex(actual), hex(expected))
                    assert actual & 0x8000, 'OS did not indicate completed conversion'
                    results.append(dict(mode='single', rate=rate, pga=pga, mux=mux,
                                        raw=raw, volts=raw * RANGES[pga] / 32768,
                                        elapsed_s=time.monotonic()-start))
            print(f'PASS single-shot {rate} SPS: 6 gains x 8 inputs', flush=True)
        c['channels'] = [dict(ch, enabled=ch['mux'] == 4) for ch in c['channels']]
        c.update(mode='continuous', pga=1)
        for rate in RATES:
            c['rate'] = rate
            adc.configure(c)
            for _ in range(5):
                start = time.monotonic()
                raw = adc.sample(c, 4)
                assert adc.read(1) & 0x7fff == config_word(c, 4)
                results.append(dict(mode='continuous', rate=rate, pga=1, mux=4, raw=raw,
                                    volts=raw * RANGES[1] / 32768, elapsed_s=time.monotonic()-start))
            adc.idle()
            assert adc.read(1) & 0x8103 == 0x8103, 'Power-down failed'
        print('PASS continuous: 8 rates, 5 reads each, power-down between tests', flush=True)
        c.update(mode='single', rate=128)
        alert_checks = 0
        for alert in ('disabled', 'traditional', 'window', 'ready'):
            for polarity in (0, 1):
                for latch in (False, True):
                    for queue in (1, 2, 4):
                        c.update(alert=alert, polarity=polarity, latch=latch, queue=queue)
                        adc.configure(c)
                        adc.sample(c, 4)
                        assert adc.read(1) & 0x7fff == config_word(c, 4)
                        alert_checks += 1
        print(f'PASS ALERT register configuration: {alert_checks} combinations (pin not measured)', flush=True)
        c = copy.deepcopy(DEFAULTS)
        adc.configure(c)
        readings = []
        for mux in range(4, 8):
            raw = adc.sample(c, mux)
            readings.append(dict(input=INPUTS[mux], raw=raw, volts=raw * RANGES[c['pga']] / 32768))
        print('Final single-ended readings:', json.dumps(readings), flush=True)
        args.report.write_text(json.dumps(dict(original=original, results=results,
                                              alert_checks=alert_checks, readings=readings), indent=2))
        print(f'PASS: {len(results)} conversions; report {args.report}', flush=True)
    finally:
        try:
            adc.idle()
            adc.write(2, original['low'])
            adc.write(3, original['high'])
            adc.write(1, original['config'] & 0x7fff)
        finally:
            adc.bus.close()


if __name__ == '__main__':
    main()
