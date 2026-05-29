#!/usr/bin/env python3
# plots.py — generate Phase-2 result figures from results/*.csv into figures/*.pdf
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES  = os.path.join(HERE, 'results')
FIG  = os.path.join(HERE, 'figures')
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update({
    'font.size': 11, 'axes.grid': True, 'grid.alpha': 0.3,
    'figure.dpi': 120, 'savefig.bbox': 'tight', 'axes.axisbelow': True,
})
C1, C2, C3 = '#1f4e9b', '#c0392b', '#1e8449'

def load(name):
    with open(os.path.join(RES, name)) as f:
        r = list(csv.reader(f))
    return r[0], [[float(x) if _isnum(x) else x for x in row] for row in r[1:]]

def _isnum(x):
    try: float(x); return True
    except: return False

def col(rows, i): return [r[i] for r in rows]

# ---- Fig 1: baseline MEI time series ----
h, rows = load('exp1_mei_timeseries.csv')
step, mei, sd = np.array(col(rows,0)), np.array(col(rows,1)), np.array(col(rows,2))
fig, ax = plt.subplots(figsize=(6,3.6))
ax.plot(step, mei, color=C1, lw=2, label='MEI (mean of 5 seeds)')
ax.fill_between(step, mei-sd, mei+sd, color=C1, alpha=0.2, label=r'$\pm 1\sigma$')
ax.axhline(1.0, color='gray', ls='--', lw=1, label='ideal circle (MEI=1)')
ax.axvline(646.6, color=C2, ls=':', lw=1.5, label='light reached (~647 steps)')
ax.set_xlabel('simulation step'); ax.set_ylabel('MEI'); ax.set_ylim(0.6, 1.3)
ax.legend(fontsize=8.5, loc='upper right')
fig.savefig(os.path.join(FIG,'fig_exp1_baseline.pdf')); plt.close(fig)

# ---- Fig 2: friction sweep (dual axis) ----
h, rows = load('exp2_friction.csv')
g = np.array(col(rows,0)); mei=np.array(col(rows,1)); meisd=np.array(col(rows,2)); nav=np.array(col(rows,3))
fig, ax = plt.subplots(figsize=(6,3.6))
ax.errorbar(g, mei, yerr=meisd, color=C1, marker='o', ms=4, lw=1.8, label='mean MEI')
ax.set_xlabel(r'friction coefficient $\gamma$'); ax.set_ylabel('mean MEI (steps 500–end)', color=C1)
ax.tick_params(axis='y', labelcolor=C1)
ax2 = ax.twinx(); ax2.grid(False)
ax2.plot(g, nav, color=C2, marker='s', ms=4, lw=1.8, label='navigation time')
ax2.set_ylabel('navigation time (steps)', color=C2); ax2.tick_params(axis='y', labelcolor=C2)
ax.axhline(1.0, color='gray', ls='--', lw=1)
fig.savefig(os.path.join(FIG,'fig_exp2_friction.pdf')); plt.close(fig)

# ---- Fig 3: noise robustness (dual axis) ----
h, rows = load('exp3_noise.csv')
s=np.array(col(rows,0)); mei=np.array(col(rows,1)); meisd=np.array(col(rows,2)); nav=np.array(col(rows,3))
fig, ax = plt.subplots(figsize=(6,3.6))
ax.errorbar(s, mei, yerr=meisd, color=C1, marker='o', ms=4, lw=1.8)
ax.set_xlabel(r'sensor noise level $\sigma$'); ax.set_ylabel('mean MEI (steps 500–1000)', color=C1)
ax.tick_params(axis='y', labelcolor=C1)
ax2 = ax.twinx(); ax2.grid(False)
ax2.plot(s, nav, color=C2, marker='s', ms=4, lw=1.8)
ax2.set_ylabel('navigation time (steps)', color=C2); ax2.tick_params(axis='y', labelcolor=C2)
ax.axhline(1.0, color='gray', ls='--', lw=1)
fig.savefig(os.path.join(FIG,'fig_exp3_noise.pdf')); plt.close(fig)

# ---- Fig 4: dynamic light tracking ----
h, rows = load('exp4_tracking.csv')
w=np.array(col(rows,0)); err=np.array(col(rows,1)); errsd=np.array(col(rows,2))
fig, ax = plt.subplots(figsize=(6,3.6))
ax.errorbar(w, err, yerr=errsd, color=C3, marker='o', ms=4, lw=1.8)
ax.axhline(20, color='gray', ls='--', lw=1, label='oscillation amplitude (A=20)')
ax.set_xlabel(r'light angular speed $\omega$ (rad/step)')
ax.set_ylabel('steady-state tracking error (world units)')
ax.legend(fontsize=9)
fig.savefig(os.path.join(FIG,'fig_exp4_tracking.pdf')); plt.close(fig)

# ---- Fig 5: failure & recovery + fatigue ----
h, rec = load('exp5a_recovery.csv')
ev = np.array([r[1] for r in rec]); rt = np.array([r[4] for r in rec])
means = [rt[ev==e].mean() for e in (1,2,3)]
fig, (axA, axB) = plt.subplots(1, 2, figsize=(9,3.6))
axA.bar([1,2,3], means, color=C1, alpha=0.8, width=0.5)
for e in (1,2,3):
    axA.scatter(np.full((ev==e).sum(), e), rt[ev==e], color=C2, s=18, zorder=3)
axA.set_xticks([1,2,3]); axA.set_xlabel('kill-20% event #')
axA.set_ylabel('MEI recovery time (steps)')
axA.set_title('(a) recovery after sudden 20% loss')
# fatigue: average active over seeds
h, fat = load('exp5b_fatigue.csv')
steps = sorted(set(r[1] for r in fat))
act = [np.mean([r[2] for r in fat if r[1]==st]) for st in steps]
dth = [np.mean([r[3] for r in fat if r[1]==st]) for st in steps]
axB.plot(steps, act, color=C1, marker='o', ms=3, lw=1.8, label='active modules')
axB.set_xlabel('simulation step'); axB.set_ylabel('active modules', color=C1)
axB.tick_params(axis='y', labelcolor=C1); axB.set_title('(b) collision-fatigue equilibrium')
axB2 = axB.twinx(); axB2.grid(False)
axB2.plot(steps, dth, color=C2, marker='s', ms=3, lw=1.8, label='cumulative deaths')
axB2.set_ylabel('cumulative deaths', color=C2); axB2.tick_params(axis='y', labelcolor=C2)
fig.savefig(os.path.join(FIG,'fig_exp5_failure.pdf')); plt.close(fig)

# ---- Fig 6: heterogeneous modules ----
h, r1 = load('exp1_mei_timeseries.csv')
h, r6 = load('exp6_het_mei_timeseries.csv')
st=np.array(col(r1,0))
fig, (axA, axB) = plt.subplots(1, 2, figsize=(9,3.6))
axA.plot(st, col(r1,1), color=C1, lw=2, label='homogeneous')
axA.plot(np.array(col(r6,0)), col(r6,1), color=C2, lw=2, label='heterogeneous (S/M/L)')
axA.axhline(1.0, color='gray', ls='--', lw=1)
axA.set_xlabel('simulation step'); axA.set_ylabel('MEI'); axA.legend(fontsize=9)
axA.set_title('(a) shape: homogeneous vs heterogeneous')
with open(os.path.join(RES,'summary.json')) as f: S=json.load(f)
e6=S['exp6']
tiers=['Small\n(r=0.30)','Medium\n(r=0.50)','Large\n(r=0.75)']
dists=[e6['sort_dist_S'],e6['sort_dist_M'],e6['sort_dist_L']]
axB.bar(tiers, dists, color=[C3,C1,C2], alpha=0.85)
axB.set_ylabel('mean distance to swarm center (world units)')
axB.set_title('(b) size sorting (last 500 steps)')
fig.savefig(os.path.join(FIG,'fig_exp6_het.pdf')); plt.close(fig)

# ---- Fig 7: single-slit gap traversal ----
h, rows = load('exp7_singleslit.csv')
gap=np.array(col(rows,0)); mx=np.array(col(rows,2)); fin=np.array(col(rows,3))
fig, ax = plt.subplots(figsize=(6,3.6))
ax.plot(gap, mx, color=C2, marker='o', ms=5, lw=1.8, label='max MEI during traversal')
ax.plot(gap, fin, color=C1, marker='s', ms=5, lw=1.8, label='final MEI (step 3000)')
ax.axhline(1.0, color='gray', ls='--', lw=1, label='ideal circle')
ax.invert_xaxis()
ax.set_xlabel('gap width (world units)'); ax.set_ylabel('MEI')
ax.legend(fontsize=9); ax.set_title('all gap widths: 100% traversal success')
fig.savefig(os.path.join(FIG,'fig_exp7_gap.pdf')); plt.close(fig)

# ---- Fig 8: double-slit ----
h, rows = load('exp8_doubleslit.csv')
two=[r for r in rows if r[1]==2]; one=[r for r in rows if r[1]==1]
fig, ax = plt.subplots(figsize=(6,3.6))
x=np.arange(2); width=0.35
two_split=np.mean([r[5] for r in two]); one_split=np.mean([r[5] for r in one])
two_cr=np.mean([r[4] for r in two]); one_cr=np.mean([r[4] for r in one])
ax.bar(x-width/2,[two_split,two_cr],width,color=C1,alpha=0.85,label='two lights')
ax.bar(x+width/2,[one_split,one_cr],width,color=C2,alpha=0.85,label='single centered light')
ax.set_xticks(x); ax.set_xticklabels(['split balance index','fraction crossed'])
ax.set_ylim(0,1.05); ax.set_ylabel('value'); ax.legend(fontsize=9)
ax.set_title('double-slit: swarm divides evenly through both gaps')
fig.savefig(os.path.join(FIG,'fig_exp8_doubleslit.pdf')); plt.close(fig)

print('Wrote 8 figures to', FIG)
print('  files:', sorted(os.listdir(FIG)))
