#!/usr/bin/env python3
"""Overlay measured turnarounds, including pauses hidden by full-sweep plots."""
import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

path,output=sys.argv[1:]
rows=list(csv.DictReader(open(path)))
names=list(dict.fromkeys(r['trial'] for r in rows))
fig,axes=plt.subplots(len(names),2,figsize=(12,3.5*len(names)),squeeze=False)
for i,name in enumerate(names):
    a=[r for r in rows if r['trial']==name]
    t=np.array([float(r['elapsed']) for r in a])
    p=np.array([float(r['position']) for r in a])*360/4096
    v=np.array([float(r['velocity']) for r in a])
    g=np.array([float(r['goal']) for r in a])*360/4096
    lo,hi=min(g),max(g)
    reversals=[k for k in range(1,len(t)) if g[k]!=g[k-1] and g[k] in (lo,hi) and g[k-1] in (lo,hi)]
    for j,k in enumerate(reversals[:-1]):
        direction=1 if g[k]>g[k-1] else -1
        mask=(t>=t[k]-1)&(t<=t[k]+1)
        axes[i,0].plot(t[mask]-t[k],(p[mask]-p[k])*direction,label=f'Reversal {j+1}',alpha=.8)
        axes[i,1].plot(t[mask]-t[k],v[mask]*direction,label=f'Reversal {j+1}',alpha=.8)
    axes[i,0].set_title(f'{name}: encoder movement at reversal')
    axes[i,0].set_ylabel('Angle from reversal position (degrees)')
    axes[i,1].set_title('Raw velocity: near-zero interval shows hesitation')
    axes[i,1].set_ylabel('Speed in new direction (degrees/s)')
    for ax in axes[i]:
        ax.axvline(0,color='black',lw=1,ls='--')
        ax.axhline(0,color='gray',lw=.5)
        ax.set_xlim(-1,1)
        ax.set_xlabel('Time relative to endpoint-command reversal (s)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
fig.suptitle('Turnaround audit — measured position and raw velocity\nDirection normalized; no temporal smoothing applied',fontsize=14)
fig.tight_layout(rect=(0,0,1,.94))
fig.savefig(output,dpi=150)
