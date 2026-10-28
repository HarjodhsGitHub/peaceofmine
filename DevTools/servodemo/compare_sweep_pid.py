#!/usr/bin/env python3
"""Compare cruise jitter under several stopped-only PID settings; restore original.

Run in the same ROS environment as tune_pid.py. This deliberately does not keep
any candidate automatically: complete motion records are needed for comparison.
"""
import argparse
import csv
import json
import statistics
import time
from pathlib import Path
from tune_pid import Abort, Experiment, Limits, RosGate, ROOT
from peaceofmine_operator.servo_transport import ArbotiX


def jitter(path):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    points=[]
    for k,row in enumerate(rows):
        if k < 12 or k+2 >= len(rows):
            continue
        previous=rows[k-12:k+1]
        if all(int(r['speed_limit'])==29 and abs(float(r['goal'])-float(r['position']))>228 for r in previous):
            a,b=rows[k-2],rows[k+2]
            slope=abs((float(b['position'])-float(a['position']))*360/4096/(float(b['elapsed'])-float(a['elapsed'])))
            direction = 1 if float(row['goal'])>float(row['position']) else -1
            points.append((float(row['velocity'])*direction,slope,direction))
    if len(points)<50:
        raise Abort('Insufficient sustained cruise for jitter comparison')
    raw,slope,directions=zip(*points)
    directional = {}
    for direction in (-1,1):
        selected = [v for v,_,d in points if d==direction]
        if len(selected)>20:
            encoder = [v for _,v,d in points if d==direction]
            directional[str(direction)] = dict(samples=len(selected), mean=statistics.mean(selected),
                sd=statistics.pstdev(selected), cv_percent=100*statistics.pstdev(selected)/statistics.mean(selected),
                encoder_sd=statistics.pstdev(encoder))
    return dict(by_direction=directional,samples=len(points), raw_speed_sd=statistics.pstdev(raw),
                raw_speed_mean=statistics.mean(raw), encoder_speed_sd=statistics.pstdev(slope),
                stopped_fraction=sum(abs(v)<3 for v in raw)/len(raw),
                wrong_direction_fraction=sum(v < -3 for v in raw)/len(raw))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--candidate', action='append', help='Bounded P,I,D candidate; may be repeated')
    p.add_argument('--reversals', type=int, default=3)
    args=p.parse_args()
    candidates=((32,0,0),(48,0,0),(48,0,8),(64,0,8),(48,0,16),(64,0,16))
    if args.candidate:
        candidates = tuple(tuple(int(v) for v in item.split(',')) for item in args.candidate)
        if any(len(g)!=3 or not 1<=g[0]<=96 or not 0<=g[1]<=2 or not 0<=g[2]<=64 for g in candidates):
            raise ValueError('Comparison candidates require P=1..96, I=0..2, D=0..64')
    if not 3 <= args.reversals <= 12:
        raise ValueError('Reversals must be 3..12')
    if not args.run:
        print(candidates)
        return
    limits=Limits(ROOT/'src/peaceofmine_operator/operator_config.json')
    gate=RosGate('self')
    bus=None
    original=None
    results=[]
    args.output.mkdir(parents=True,exist_ok=False)
    def apply(gains):
        bus.write(253,82,0,1)
        if bus.read(limits.ident,24,1)!=0:
            raise Abort('Torque-off confirmation failed')
        bus.write(253,81,limits.ident,1)
        for address,value in zip((28,27,26),gains):
            bus.write(limits.ident,address,value,1)
            if bus.read(limits.ident,address,1)!=value:
                raise Abort('Gain readback failed')
    try:
        deadline=time.monotonic()+12
        while not gate.status()['allowed'] and time.monotonic()<deadline:
            time.sleep(.02)
        gate.check()
        bus=ArbotiX('/dev/ttyUSB0')
        original=tuple(bus.read(limits.ident,a,1) for a in (28,27,26))
        for gains in candidates:
            gate.check()
            limits.unchanged()
            apply(gains)
            output=args.output/f'p{gains[0]}-i{gains[1]}-d{gains[2]}'
            experiment=Experiment(bus,limits,gate,output)
            try:
                experiment.verify_sweep(wide=True,speeds=(29,),reversals=args.reversals)
                result=dict(gains=gains,**jitter(output/'samples.csv'))
            except RuntimeError as exc:
                # A controller settling failure is a rejected candidate.
                # Interlock/health/transport failures abort the whole suite.
                if 'firmware_fault=10' not in str(exc):
                    raise
                result=dict(gains=gains,rejected=str(exc))
            results.append(result)
            (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(result),flush=True)
    finally:
        if bus:
            try:
                bus.write(253,82,0,1)
                if original is not None:
                    apply(original)
            finally:
                bus.close()
        gate.close()


if __name__=='__main__':
    main()
