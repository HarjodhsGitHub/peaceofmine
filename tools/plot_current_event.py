#!/usr/bin/env python3
"""Graph the recorded validation abort; do not remove or smooth its samples."""
import csv
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rows=list(csv.DictReader(open(sys.argv[1])))
t=[float(r['elapsed']) for r in rows]
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
axes[0].plot(t,[float(r['position']) for r in rows],color='#087e8b')
axes[0].set_ylabel('Encoder position (ticks)')
axes[1].plot(t,[float(r['velocity']) for r in rows],color='#7751ae',label='Raw measured velocity')
axes[1].plot(t,[float(r['speed_limit'])*.684 for r in rows],'--',color='#d78d22',label='Nominal speed limit')
axes[1].set_ylabel('Velocity (degrees/s)')
axes[1].legend()
axes[2].plot(t,[abs(float(r['current'])) for r in rows],color='#c23d3d',label='Absolute current')
axes[2].axhline(1.5,color='black',ls='--',label='Test abort threshold')
axes[2].set_ylabel('Current (A)')
axes[2].set_xlabel('Time since sweep start (s)')
axes[2].legend()
for ax in axes:
    ax.grid(alpha=.2)
    ax.axvline(t[-1],color='#c23d3d',ls=':')
    ax.set_xlim(max(0,t[-1]-2),t[-1]+.1)
fig.suptitle('Final validation stopped at 1.61 A — P=48, I=0, D=16\nRising current and irregular movement occurred before the stop',fontsize=14)
fig.tight_layout(rect=(0,0,1,.93))
fig.savefig(sys.argv[2],dpi=150)
