//
// verify.js — correctness checks for sim_core.js before running experiments.
//
'use strict';
const { Sim } = require('./sim_core.js');

let pass = 0, fail = 0;
function check(name, cond, info = '') {
    if (cond) { pass++; console.log('  PASS  ' + name + (info ? '  (' + info + ')' : '')); }
    else      { fail++; console.log('  FAIL  ' + name + '  ' + info); }
}

console.log('=== 1. Determinism ===');
{
    const a = new Sim({ seed: 7 }); a.initCluster();
    const b = new Sim({ seed: 7 }); b.initCluster();
    let same = a.modules.length === b.modules.length;
    let mA, mB;
    for (let t = 0; t < 200; t++) { mA = a.step(); mB = b.step(); }
    check('same seed -> identical MEI', same && Math.abs(mA - mB) < 1e-12, 'MEI=' + mA.toFixed(6));
    const c = new Sim({ seed: 8 }); c.initCluster();
    let mC; for (let t = 0; t < 200; t++) mC = c.step();
    check('different seed -> different MEI', Math.abs(mA - mC) > 1e-6, 'd=' + Math.abs(mA - mC).toExponential(2));
}

console.log('=== 2. MEI on an ideal hex-packed disc ===');
{
    // build a hex lattice of touching circles (spacing = 2r) inside a disc
    const r = 0.5, spacing = 2 * r;
    const pts = [];
    const R = 5.0;
    for (let row = -12; row <= 12; row++) {
        const y = row * spacing * Math.sqrt(3) / 2;
        const xoff = (row % 2) ? spacing / 2 : 0;
        for (let col = -12; col <= 12; col++) {
            const x = col * spacing + xoff;
            if (x * x + y * y <= R * R) pts.push({ x: x + 70, y: y + 30 });
        }
    }
    const s = new Sim({ seed: 1 });
    s.modules = pts.map(p => ({ x: p.x, y: p.y, vx: 0, vy: 0, active: true,
        inShadow: false, inShadow2: false, radius: r, collisions: 0, lastCollisionStep: 0,
        tokens: new Int32Array(22), potential: 0 }));
    const meiCode = s.computeMEI();
    console.log('  N=' + s.modules.length + '  MEI(corrected formula)=' + meiCode.toFixed(3));
    check('ideal hex-packed disc -> MEI ~ 1.0', Math.abs(meiCode - 1.0) < 0.05,
          'MEI=' + meiCode.toFixed(3));
}

console.log('=== 3. Potential gradient (center > edge) + potential-driven cohesion ===');
{
    // No light: with all-neighbor diffusion the gradient is smooth & center-high.
    const s = new Sim({ seed: 3, LIGHT_ON: false }); s.initCluster();
    for (let t = 0; t < 150; t++) s.step();
    const c = s.centerOfGravity();
    const ds = s.modules.filter(m => m.active)
        .map(m => ({ d: Math.hypot(m.x - c.x, m.y - c.y), p: m.potential }))
        .sort((a, b) => a.d - b.d);
    const k = Math.floor(ds.length / 4);
    const innerMean = ds.slice(0, k).reduce((a, x) => a + x.p, 0) / k;
    const outerMean = ds.slice(-k).reduce((a, x) => a + x.p, 0) / k;
    check('inner-quarter potential > outer-quarter', innerMean > outerMean * 1.2,
          'inner=' + innerMean.toFixed(2) + ' outer=' + outerMean.toFixed(2));

    // potential-driven cohesion: with NO light, attraction p=8 must keep the
    // swarm compact (Ishida Fig.6). p=0 (no attraction) stays loose.
    function noLightMEI(p) {
        const x = new Sim({ seed: 1, LIGHT_ON: false, P_ATTRACT: p }); x.initCluster();
        let m; for (let t = 0; t < 1500; t++) m = x.step(); return m;
    }
    const mP8 = noLightMEI(8), mP0 = noLightMEI(0);
    check('attraction compacts the swarm without light (p=8 MEI < 1.3)', mP8 < 1.3,
          'p8=' + mP8.toFixed(2));
    check('attraction p=8 more compact than p=0 (Ishida behaviour)', mP8 < mP0,
          'p8=' + mP8.toFixed(2) + ' p0=' + mP0.toFixed(2));
}

console.log('=== 4. Light pulls COG toward source ===');
{
    const s = new Sim({ seed: 4 }); s.initCluster();
    const d0 = Math.hypot(s.centerOfGravity().x - s.lightSource.x, s.centerOfGravity().y - s.lightSource.y);
    for (let t = 0; t < 600; t++) s.step();
    const d1 = Math.hypot(s.centerOfGravity().x - s.lightSource.x, s.centerOfGravity().y - s.lightSource.y);
    check('COG moves closer to light', d1 < d0 - 10, 'd0=' + d0.toFixed(1) + ' d1=' + d1.toFixed(1));
}

console.log('=== 5. Wall blocking (single module driven by a light) ===');
{
    function run(y) {
        const s = new Sim({ seed: 5, WALL_ON: true, WALL_GAP: 7, LIGHT_X: 105, LIGHT_Y: y });
        s.modules = [{ x: 60, y, vx: 0, vy: 0, active: true, inShadow: false, inShadow2: false,
            radius: 0.5, collisions: 0, lastCollisionStep: 0, tokens: new Int32Array(22), potential: 0 }];
        for (let t = 0; t < 400; t++) s.step();
        return s.modules[0].x;
    }
    check('module aimed at solid wall is blocked (x<70)', run(8) < 70, 'x=' + run(8).toFixed(2));
    check('module aimed at gap passes through (x>70)', run(30) > 70, 'x=' + run(30).toFixed(2));
}

console.log('=== 6. Kill 20% removes ~20% ===');
{
    const s = new Sim({ seed: 6 }); s.initCluster();
    const before = s.countActive();
    s.killRandom20();
    const after = s.countActive();
    const frac = (before - after) / before;
    check('~20% deactivated', Math.abs(frac - 0.2) < 0.03, 'killed ' + (before - after) + '/' + before);
}

console.log('=== 7. Heterogeneous size distribution ~40/40/20 (sampler) ===');
{
    // test the size sampler itself (cluster placement skews this by packing,
    // which is a physical effect reported separately in Experiment 6).
    const s = new Sim({ seed: 9, HET_ON: true });
    let nS = 0, nM = 0, nL = 0, N = 100000;
    for (let i = 0; i < N; i++) { const r = s._pickRadius(); if (r < 0.35) nS++; else if (r < 0.60) nM++; else nL++; }
    check('S~40%', Math.abs(nS / N - 0.40) < 0.02, (nS / N).toFixed(3));
    check('M~40%', Math.abs(nM / N - 0.40) < 0.02, (nM / N).toFixed(3));
    check('L~20%', Math.abs(nL / N - 0.20) < 0.02, (nL / N).toFixed(3));
}

console.log('=== 9. Double-slit wall (two gaps at y=22 and y=38, solid at y=30) ===');
{
    function run(y, ly) {
        const s = new Sim({ seed: 5, WALL2_ON: true, LIGHT_X: 105, LIGHT_Y: ly });
        s.modules = [{ x: 60, y, vx: 0, vy: 0, active: true, inShadow: false, inShadow2: false,
            radius: 0.5, collisions: 0, lastCollisionStep: 0, tokens: new Float64Array(22), potential: 0 }];
        for (let t = 0; t < 400; t++) s.step();
        return s.modules[0].x;
    }
    // slit centres = 30 +/- 8 = 22 and 38 (half-width 3.5)
    check('passes through upper slit (y=22)', run(22, 22) > 70, 'x=' + run(22, 22).toFixed(2));
    check('passes through lower slit (y=38)', run(38, 38) > 70, 'x=' + run(38, 38).toFixed(2));
    check('blocked at solid middle bar (y=30)', run(30, 30) < 70, 'x=' + run(30, 30).toFixed(2));
    check('blocked at solid top (y=8)',        run(8, 8)  < 70, 'x=' + run(8, 8).toFixed(2));
}

console.log('=== 10. Second light source pulls swarm (light 1 off, light 2 on) ===');
{
    const s = new Sim({ seed: 4, LIGHT_ON: false, LIGHT2_ON: true, LIGHT2_X: 105, LIGHT2_Y: 30 });
    s.initCluster();
    const x0 = s.centerOfGravity().x;
    for (let t = 0; t < 800; t++) s.step();
    const c = s.centerOfGravity();
    const d = Math.hypot(c.x - s.lightSource2.x, c.y - s.lightSource2.y);
    check('COG moves toward light 2', c.x > x0 + 40 && d < 10, 'x0=' + x0.toFixed(1) + ' -> x=' + c.x.toFixed(1) + ' dist2=' + d.toFixed(1));
}

console.log('=== 11. Collision-fatigue failure (E7) ===');
{
    // dense cluster + low threshold + fail ON -> deaths occur
    const dense = new Sim({ seed: 1, COLLISION_FAIL_ON: true, COLLISION_THRESHOLD: 20, N_INIT: 120, INIT_RADIUS: 4 });
    dense.initCluster();
    for (let t = 0; t < 800; t++) dense.step();
    check('dense cluster + fail-on produces deaths', dense.totalCollisionDeaths > 0,
          'deaths=' + dense.totalCollisionDeaths);

    // same dense cluster but fail OFF -> no collision deaths
    const off = new Sim({ seed: 1, COLLISION_FAIL_ON: false, COLLISION_THRESHOLD: 20, N_INIT: 120, INIT_RADIUS: 4 });
    off.initCluster();
    for (let t = 0; t < 800; t++) off.step();
    check('fail-off produces no collision deaths', off.totalCollisionDeaths === 0,
          'deaths=' + off.totalCollisionDeaths);

    // collision counter resets after an idle period (no overlap for RESET window)
    const s = new Sim({ seed: 1, COLLISION_FAIL_ON: true, COLLISION_THRESHOLD: 1000 });
    s.modules = [
        { x: 70, y: 30, vx: 0, vy: 0, active: true, inShadow: false, inShadow2: false, radius: 0.5,
          collisions: 50, lastCollisionStep: 0, tokens: new Float64Array(22), potential: 0 }];
    // isolated module, no neighbors -> after RESET_STEPS its counter must reset to 0
    for (let t = 0; t < s.p.COLLISION_RESET_STEPS + 5; t++) s.step();
    check('collision counter resets after idle window', s.modules[0].collisions === 0,
          'collisions=' + s.modules[0].collisions);
}

console.log('=== 8. Token non-negativity ===');
{
    const s = new Sim({ seed: 11, NOISE_ON: true, NOISE_SIGMA: 3 }); s.initCluster();
    let ok = true;
    for (let t = 0; t < 50; t++) {
        s.step();
        for (const m of s.modules) for (let n = 0; n < m.tokens.length; n++) if (m.tokens[n] < 0) ok = false;
    }
    check('no negative token counts even with heavy noise', ok);
}

console.log('\n=== RESULT: ' + pass + ' passed, ' + fail + ' failed ===');
process.exit(fail ? 1 : 0);
