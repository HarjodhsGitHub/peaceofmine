#!/usr/bin/env python3
"""Measure shared A0/A3 acquisition. Stop the operator before using physical I²C."""
import argparse
import copy
import json
import time
from peaceofmine_operator.ads1115 import DEFAULTS
from peaceofmine_operator.adc_acquisition import Acquisition


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bus',type=int,default=1)
    parser.add_argument('--address',type=lambda s:int(s,0),default=0x48)
    parser.add_argument('--rate',type=int,default=860)
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--required-a3-hz',type=float,required=True)
    parser.add_argument('--simulation',action='store_true')
    args=parser.parse_args()
    if args.seconds < 1 or args.required_a3_hz <= 0:
        parser.error('duration must be >= 1 second and required rate must be positive')
    config=copy.deepcopy(DEFAULTS)
    config.update(bus=args.bus,address=args.address,rate=args.rate,interval_ms=0)
    for channel in config['channels']: channel['enabled']=channel['mux'] in (4,7)
    adc=Acquisition(args.simulation,config);adc.thread.start();adc.run(True)
    samples={4:[],7:[]};cursor=0;end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:
            state=adc.snapshot(cursor)
            for sample in state['samples']:
                samples[sample['mux']].append(sample['time']);cursor=sample['seq']
            time.sleep(.02)
        report={}
        for mux,times in samples.items():
            gaps=[b-a for a,b in zip(times,times[1:])]
            report['A0' if mux==4 else 'A3']=dict(samples=len(times),
                effective_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
                max_gap_s=max(gaps) if gaps else None)
        passed=bool(state['connected'] and report['A3']['effective_hz']>=args.required_a3_hz)
        print(json.dumps(dict(simulation=args.simulation,bus=args.bus,address=hex(args.address),
            configured_adc_rate=args.rate,required_a3_hz=args.required_a3_hz,
            sampling_passed=passed,error=state['error'],channels=report),indent=2))
        return 0 if passed else 1
    finally: adc.close()


if __name__=='__main__': raise SystemExit(main())
