#!/usr/bin/env python3
"""Plot raw servo telemetry and quantify cruise ripple without hiding reversals."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(path, output):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    names = list(dict.fromkeys(r['trial'] for r in rows))
    fig, axes = plt.subplots(len(names), 3, figsize=(17, 4*len(names)), squeeze=False)
    summary = {}
    for index, name in enumerate(names):
        data = {key:np.array([float(r[key]) for r in rows if r['trial']==name])
                for key in ('elapsed','position','velocity','goal','speed_limit','current')}
        t, position, velocity = data['elapsed'], data['position']*360/4096, data['velocity']
        goal = data['goal']*360/4096
        cap = data['speed_limit']*.684
        direction = np.sign(goal-position)
        # Centered 5-sample encoder slope confirms register-speed variations,
        # while raw speed remains visible. It is not used to invent samples.
        derived = np.full(len(t), np.nan)
        for k in range(2,len(t)-2):
            derived[k] = np.polyfit(t[k-2:k+3]-t[k],position[k-2:k+3],1)[0]
        requested = int(name.split('-')[-1])*.684
        cruise = (cap >= requested-.7)&(np.abs(goal-position)>20)
        # Omit first 0.5 s after cap attainment for acceleration transients.
        stable = cruise.copy()
        for k in range(len(t)):
            stable[k] = cruise[k] and t[k]-t[max(0,k-12)] >= .4 and np.all(cruise[max(0,k-12):k+1])
        endpoints = (float(np.min(data['goal'])), float(np.max(data['goal'])))
        switches = np.array([k for k in range(1,len(t)) if data['goal'][k] != data['goal'][k-1]
                             and data['goal'][k] in endpoints and data['goal'][k-1] in endpoints],dtype=int)
        values = np.abs(velocity[stable])
        derived_values = np.abs(derived[stable & np.isfinite(derived)])
        pauses=[]
        start=None
        for k in range(len(t)):
            paused = abs(velocity[k])<3 and abs(goal[k]-position[k])>5
            if paused and start is None: start=t[k]
            if not paused and start is not None:
                if t[k]-start >= .1: pauses.append(t[k]-start)
                start=None
        stats=dict(samples=len(t), sample_rate_hz=(len(t)-1)/(t[-1]-t[0]),
                   reversals=int(len(switches)), requested_deg_s=requested,
                   max_actual_deg_s=float(np.max(np.abs(velocity))),
                   cruise_samples=int(len(values)), pauses_over_100ms=pauses)
        if len(values):
            stats.update(cruise_mean_deg_s=float(np.mean(values)), cruise_sd_deg_s=float(np.std(values)),
                         cruise_cv_percent=float(100*np.std(values)/np.mean(values)),
                         encoder_slope_sd_deg_s=float(np.std(derived_values)),
                         cruise_stopped_fraction=float(np.mean(values<3)))
        summary[name]=stats
        ax=axes[index,0]
        ax.plot(t,goal,'--',color='#89939d',lw=1,label='Endpoint command')
        ax.plot(t,position,color='#0a9d88',lw=1.4,label='Measured position')
        ax.set_ylabel('Position (degrees, raw encoder zero)')
        ax.set_title(f'{name}: position and reversals')
        for point in t[switches]: ax.axvline(point,color='#aaaaaa',lw=.5,alpha=.5)
        ax.legend(fontsize=8)
        ax=axes[index,1]
        ax.plot(t,velocity,color='#725bb0',lw=.7,alpha=.75,label='Raw measured velocity')
        ax.plot(t,derived,color='#167c9b',lw=1,label='Encoder slope (~0.15 s window)')
        ax.plot(t,cap*direction,color='#e4952e',lw=.9,label='Signed speed limit')
        ax.set_ylabel('Velocity (degrees/s)')
        ax.set_title('Raw speed: bumps remain visible')
        ax.legend(fontsize=8)
        # Zoom first established cruise, or center of first leg when no cruise.
        chosen=np.flatnonzero(stable)
        left=t[chosen[0]] if len(chosen) else min(1.,t[-1]/3)
        ax=axes[index,2]
        ax.plot(t,velocity,color='#725bb0',marker='.',ms=3,lw=.8,label='Raw speed')
        ax.plot(t,derived,color='#167c9b',lw=1.2,label='Encoder slope')
        ax.plot(t,cap*direction,color='#e4952e',lw=1,label='Speed limit')
        ax.set_xlim(left,min(left+2.5,t[-1]))
        ax.set_ylabel('Velocity (degrees/s)')
        label=f"Cruise ripple: {stats['cruise_cv_percent']:.1f}% SD/mean" if len(values) else 'No sustained maximum-speed cruise'
        ax.set_title(label)
        ax.legend(fontsize=8)
        for ax in axes[index]:
            ax.grid(alpha=.2)
            ax.set_xlabel('Time (s)')
    gains = ', '.join(f'{key.upper()}={rows[0][key]}' for key in ('p','i','d'))
    fig.suptitle(f'Measured servo sweep — {gains}\nDashed endpoints are commands, not desired continuous trajectories',fontsize=15)
    fig.tight_layout(rect=(0,0,1,.95))
    output=Path(output)
    fig.savefig(output,dpi=150)
    output.with_suffix('.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('output')
    args=p.parse_args()
    analyze(args.csv,args.output)
