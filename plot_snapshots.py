#!/usr/bin/env python3
# plot_snapshots.py — qualitative swarm snapshots from results/snapshots.json
import json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

HERE = os.path.dirname(os.path.abspath(__file__))
FIG  = os.path.join(HERE, 'figures')
os.makedirs(FIG, exist_ok=True)
with open(os.path.join(HERE,'results','snapshots.json')) as f:
    D = json.load(f)
W, H, WX = D['world']['w'], D['world']['h'], D['world']['wallX']

def draw_panel(ax, cap, scen, gap=None, double=False):
    ax.set_facecolor('#0a1628')
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([])
    # wall
    if scen in ('singleslit','doubleslit'):
        gh = (gap or 7)/2
        if double:
            sep=16; cy=30
            segs=[(0, cy-sep/2-gh),(cy-sep/2+gh, cy+sep/2-gh),(cy+sep/2+gh, H)]
        else:
            segs=[(0,30-gh),(30+gh,H)]
        for y0,y1 in segs:
            ax.add_patch(plt.Rectangle((WX-0.7,y0),1.4,y1-y0,color='#2a7a50'))
    # light(s)
    for L,c in [(cap['light'],'#ffd23f'),(cap['light2'],'#40e0ff')]:
        if L['on']:
            ax.scatter([L['x']],[L['y']],marker='*',s=180,color=c,edgecolors='k',linewidths=0.4,zorder=5)
    mods=cap['modules']
    act=[m for m in mods if m['a']]
    dor=[m for m in mods if not m['a']]
    if act:
        ps=np.array([m['p'] for m in act]);
        lo,hi=np.percentile(ps,5),np.percentile(ps,95); hi=max(hi,lo+1e-6)
        circ=[Circle((m['x'],m['y']),m['r']) for m in act]
        pc=PatchCollection(circ,cmap='turbo'); pc.set_array(np.clip((ps-lo)/(hi-lo),0,1)); pc.set_zorder(3)
        ax.add_collection(pc)
    if dor:
        circ=[Circle((m['x'],m['y']),m['r']) for m in dor]
        pc=PatchCollection(circ,facecolor='#5a5a6e',zorder=2); ax.add_collection(pc)
    ax.set_title(f"t={cap['step']}, MEI={cap['mei']}", fontsize=9, color='k', pad=3)

def figure(scen, double=False, gap=None, fname=None):
    caps=D[scen]['caps']
    n=len(caps)
    fig,axes=plt.subplots(1,n,figsize=(3.0*n,1.55))
    if n==1: axes=[axes]
    for ax,cap in zip(axes,caps):
        draw_panel(ax,cap,scen,gap=gap,double=double)
    fig.subplots_adjust(wspace=0.06,left=0.01,right=0.99,top=0.86,bottom=0.02)
    fig.savefig(os.path.join(FIG,fname),dpi=140,bbox_inches='tight'); plt.close(fig)
    print('wrote',fname)

figure('baseline', fname='fig_snap_baseline.pdf')
figure('singleslit', gap=D['singleslit']['gap'], fname='fig_snap_singleslit.pdf')
figure('doubleslit', double=True, fname='fig_snap_doubleslit.pdf')

# potential field single panel (with colorbar)
cap=D['potential']['caps'][0]
fig,ax=plt.subplots(figsize=(4.2,2.2))
draw_panel(ax,cap,'potential')
act=[m for m in cap['modules'] if m['a']]
ps=np.array([m['p'] for m in act])
sc=ax.scatter([m['x'] for m in act],[m['y'] for m in act],c=ps,cmap='turbo',s=8,zorder=4)
cb=fig.colorbar(sc,ax=ax,fraction=0.025,pad=0.02); cb.set_label('potential',fontsize=8); cb.ax.tick_params(labelsize=7)
# zoom to cluster
xs=[m['x'] for m in act]; ys=[m['y'] for m in act]
ax.set_xlim(min(xs)-3,max(xs)+3); ax.set_ylim(min(ys)-3,max(ys)+3)
ax.set_title('Settled cluster potential (no light): high at center',fontsize=9,color='k')
fig.savefig(os.path.join(FIG,'fig_snap_potential.pdf'),dpi=140,bbox_inches='tight'); plt.close(fig)
print('wrote fig_snap_potential.pdf')
