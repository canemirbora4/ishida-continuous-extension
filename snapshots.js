//
// snapshots.js — dump module positions at chosen steps for qualitative figures.
// Writes results/snapshots.json
//
'use strict';
const fs = require('fs');
const path = require('path');
const { Sim } = require('./sim_core.js');

function snap(s) {
    return {
        step: s.iterate,
        light: { x: s.lightSource.x, y: s.lightSource.y, on: s.p.LIGHT_ON },
        light2: { x: s.lightSource2.x, y: s.lightSource2.y, on: s.p.LIGHT2_ON },
        mei: +s.computeMEI().toFixed(3),
        modules: s.modules.map(m => ({
            x: +m.x.toFixed(2), y: +m.y.toFixed(2), a: m.active ? 1 : 0,
            r: m.radius, p: +m.potential.toFixed(2),
        })),
    };
}

function runCapture(cfg, captureSteps, maxStep) {
    const s = new Sim(cfg);
    s.initCluster();
    const caps = [];
    if (captureSteps.includes(0)) caps.push(snap(s));
    for (let t = 1; t <= maxStep; t++) {
        s.step();
        if (captureSteps.includes(t)) caps.push(snap(s));
    }
    return caps;
}

const out = {
    world: { w: 140, h: 60, wallX: 70 },
    baseline:  { caps: runCapture({ seed: 1 }, [0, 150, 400, 700], 700),
                 desc: 'Baseline navigation: cluster forms and moves to the light.' },
    singleslit:{ caps: runCapture({ seed: 1, WALL_ON: true, WALL_GAP: 4 }, [0, 500, 700, 1400], 1400),
                 gap: 4, desc: 'Single-slit (gap=4): swarm squeezes through and reforms.' },
    doubleslit:{ caps: runCapture({ seed: 1, WALL2_ON: true, LIGHT_X: 105, LIGHT_Y: 30 },
                 [0, 600, 1200, 2200], 2200),
                 desc: 'Double-slit, single centered light: swarm divides through both gaps.' },
    potential: { caps: runCapture({ seed: 1, LIGHT_ON: false }, [200], 200),
                 desc: 'Potential field of a settled cluster (no light): center-high gradient.' },
};

fs.writeFileSync(path.join(__dirname, 'results', 'snapshots.json'), JSON.stringify(out));
console.log('wrote results/snapshots.json',
    '(baseline MEIs:', out.baseline.caps.map(c => c.mei).join(', ') + ')');
