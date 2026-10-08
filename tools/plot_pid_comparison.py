#!/usr/bin/env python3
"""Compare recorded cruise ripple and unsmoothed endpoint motion at matched speeds."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from compare_sweep_pid import jitter


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before',type=Path)
    parser.add_argument('after',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    fig,axes=plt.subplots(2,2,figsize=(14,8))
    stats={}
    for row,(label,path) in enumerate((('Before damping',args.before),('Candidate — not saved',args.after))):
        with path.open() as stream:
            records=[r for r in csv.DictReader(stream) if r['trial']=='sweep-29']
        t=np.array([float(r['elapsed']) for r in records])
        pos=np.array([float(r['position']) for r in records])*360/4096
        velocity=np.array([float(r['velocity']) for r in records])
        goal=np.array([float(r['goal']) for r in records])*360/4096
        cap=np.array([float(r['speed_limit']) for r in records])*.684
        pid=', '.join(f'{k.upper()}={records[0][k]}' for k in ('p','i','d'))
        # Show the same increasing-angle stretch on both datasets.
        candidates=np.flatnonzero((pos>=1300*360/4096)&(goal>pos)&(cap>19.5)&(t>1))
        start=t[candidates[0]]
        mask=(t>=start)&(t<=start+2)
        # Finite differences are deliberately unsmoothed; quantization and
        # telemetry timestamp uncertainty also contribute to these differences.
        derivative=np.diff(pos)/np.diff(t)
        ax=axes[row,0]
        ax.plot(t[mask]-start,velocity[mask],'.-',color='#8e5ac2',lw=.8,label='Raw speed register')
        dm=(t[1:]>=start)&(t[1:]<=start+2)
        ax.plot(t[1:][dm]-start,derivative[dm],'.-',color='#087e8b',lw=.8,label='Encoder Δangle/Δtime (unsmoothed)')
        ax.axhline(29*.684,color='#d78d22',ls='--',label='Nominal speed limit')
        ax.set_title(f'{label}: {pid}\nSame direction and angle region, nominal 20°/s')
        ax.set_ylabel('Measured velocity (degrees/s)')
        ax.set_xlabel('Time in matched cruise segment (s)')
        ax.set_ylim(-10,50)
        ax.legend(fontsize=8)
        lo,hi=min(goal),max(goal)
        switches=[k for k in range(1,len(t)) if goal[k]!=goal[k-1] and goal[k] in (lo,hi) and goal[k-1] in (lo,hi)]
        ax=axes[row,1]
        holds=[]
        for n,k in enumerate(switches[:-1]):
            direction=1 if goal[k]>goal[k-1] else -1
            mask=(t>=t[k]-.2)&(t<=t[k]+.8)
            ax.plot(t[mask]-t[k],(pos[mask]-pos[k])*direction,lw=1,label=f'Reversal {n+1}')
            indices=np.flatnonzero((t>=t[k])&(t<=t[k]+.7))
            longest=0
            for a in indices:
                b=a
                while b+1<len(t) and t[b+1]<=t[k]+.7 and np.ptp(pos[a:b+2])<=2*360/4096:
                    b+=1
                longest=max(longest,t[b]-t[a])
            holds.append(longest)
        ax.axvline(0,color='black',ls='--',lw=.8)
        ax.axhline(0,color='gray',lw=.5)
        ax.set_title('Endpoint hesitation: raw encoder traces\nNew endpoint command is sent at t=0')
        ax.set_xlabel('Time since command reversal (s)')
        ax.set_ylabel('Travel from reversal position (degrees)')
        ax.set_xlim(-.2,.8)
        ax.set_ylim(-1.5,9)
        ax.legend(fontsize=8)
        stats[label]=dict(gains=pid,**jitter(path),stationary_interval_s=holds)
        for ax in axes[row]: ax.grid(alpha=.2)
    fig.suptitle('Servo jitter audit — actual recorded motion\nNo smoothing in these plots; sampling about 26 Hz limits high-frequency conclusions',fontsize=14)
    fig.tight_layout(rect=(0,0,1,.93))
    fig.savefig(args.output,dpi=160)
    args.output.with_suffix('.json').write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps(stats,indent=2))


if __name__=='__main__':
    main()
