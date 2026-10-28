#!/usr/bin/env python3
"""Validate chosen gains with repeated steps and wide sweeps; optionally save them.

Use after reviewing compare_sweep_pid.py's raw and encoder-derived jitter data.
This is a validation gate, not an exhaustive PID search.
"""
import argparse
import json
from pathlib import Path
import time
from tune_pid import Abort, Experiment, Limits, RosGate, ROOT
from peaceofmine_operator import configuration
from peaceofmine_operator.servo_pid import apply_arm_pid
from peaceofmine_operator.servo_transport import ArbotiX


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gains', required=True, help='P,I,D')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--save', action='store_true')
    args=parser.parse_args()
    gains=tuple(int(v) for v in args.gains.split(','))
    if len(gains)!=3 or not 1<=gains[0]<=96 or not 0<=gains[1]<=2 or not 0<=gains[2]<=64:
        raise ValueError('Validation gains require P=1..96, I=0..2, D=0..64')
    if not args.run:
        print('No motion: use --run after reviewing the jitter comparison')
        return
    limits=Limits(ROOT/'src/peaceofmine_operator/operator_config.json')
    initial_config=json.loads(limits.raw)
    args.output.mkdir(parents=True, exist_ok=False)
    gate=RosGate('self')
    bus=None
    original=None
    success=False
    summary=dict(gains=gains,complete=False)
    def apply(pid):
        apply_arm_pid(bus,dict(arm=dict(servo_id=limits.ident),
            arm_pid=dict(servo_id=limits.ident,p=pid[0],i=pid[1],d=pid[2])))
    try:
        deadline=time.monotonic()+12
        while not gate.status()['allowed'] and time.monotonic()<deadline:
            time.sleep(.02)
        gate.check()
        bus=ArbotiX('/dev/ttyUSB0')
        original=tuple(bus.read(limits.ident,a,1) for a in (28,27,26))
        summary['entering_gains']=original
        experiment=Experiment(bus,limits,gate,args.output/'steps')
        experiment.speed=limits.speed
        experiment.acceleration=limits.acceleration
        try:
            baseline=experiment.trial((32,0,0),'factory-baseline',limits.radius)
            candidates=[experiment.trial(gains,f'candidate-{n}',limits.radius) for n in (1,2)]
            # Jitter was assessed separately. Do not trade it for large errors,
            # oscillation, overshoot or increased current in these held-out steps.
            for result in candidates:
                if (result['tail_error']>baseline['tail_error']+5 or result['tail_noise']>3 or
                        result['overshoot']>8 or result['peak_current']>max(.8,baseline['peak_current']*1.4)):
                    raise Abort('Candidate failed repeated step acceptance criteria')
            experiment.report['complete']=True
            summary['steps']=experiment.report['trials']
        finally:
            experiment.stop()
            experiment.report['restoration']='torque off; candidate gains pending sweep validation'
            experiment.save()
            experiment.file.close()
        apply(gains)
        experiment=Experiment(bus,limits,gate,args.output/'sweeps')
        experiment.verify_sweep(wide=True)
        summary['sweeps']=experiment.report['trials']
        limits.unchanged()
        apply(gains)
        if args.save:
            profile=dict(servo_id=limits.ident,p=gains[0],i=gains[1],d=gains[2],
                         calibration=initial_config['arm'],
                         validation=str(args.output.relative_to(ROOT)))
            configuration.save_section(limits.path,'arm_pid',profile,
                                       expected_revision=initial_config.get('revision',0))
            summary['saved']=True
        success=True
        summary['complete']=True
    except BaseException as exc:
        summary['error']=str(exc)
        raise
    finally:
        if bus:
            try:
                bus.write(253,82,0,1)
                if not success and original is not None:
                    apply(original)
                summary['final_gains']=[bus.read(limits.ident,a,1) for a in (28,27,26)]
                summary['torque']=bus.read(limits.ident,24,1)
            finally:
                bus.close()
        (args.output/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
        gate.close()


if __name__=='__main__':
    main()
