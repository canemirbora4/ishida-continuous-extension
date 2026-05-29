//
// cell_robot_continuous.js
// Continuous-space extension of Ishida 2025 swarm robot algorithm.
//
// Extensions implemented:
//   E1 – Continuous Space   : free R² particles, spatial hash, force movement
//   E2 – Physical Inertia   : v += F·dt, x += v·dt, friction coefficient γ
//   E3 – 360° Direction     : implicit — forces are continuous vectors
//   E4 – Dynamic Light Source: moving / oscillating / circular target
//   E5 – Sensor Noise       : Gaussian noise on token exchange & light direction
//   E6 – Heterogeneous Modules: S/M/L radius tiers, per-module physics & shadow
//   E7 – Module Failure       : collision-count fatigue; reset if idle ~1 s
//   E8 – Multiple Light Sources: two independent lights, sequential click placement
//

// ============================================================
// World constants
// ============================================================
const WORLD_W  = 140;
const WORLD_H  = 60;
let   PX_PER_UNIT;

const R_MOD        = 0.5;   // module radius (world units)
const R_NEIGH      = 1.8;   // neighbor radius for token exchange
const DT           = 0.12;
const MAX_STEP     = 0.14;
const TOKEN_MAX    = 20;
const DIFFUSE_STEPS = 20;
const SUM2_LIMIT   = 10;    // tokens 1..10 = "inner" (N2 in paper)
const DIFFUSE_RESID = 0.3;  // residual rate: fraction of tokens kept per round (Ishida)
const HASH_CELL    = R_NEIGH;

// ============================================================
// Simulation state
// ============================================================
let canvas, ctx;
let modules  = [];
let iterate  = 0;
let running  = false;
let timer    = null;
let spatialHash = null;
let pMinEMA = 0, pMaxEMA = 1;

// Light source 1 (Extension 4 adds dynamic modes)
let lightSource    = { x: 105, y: 30 };
let lightOn        = true;
let LIGHT_MODE     = 'static';   // 'static' | 'oscillate' | 'circular'
let LIGHT_SPEED    = 0.03;
let lightAngle     = 0;
const LIGHT_CENTER = { x: 105, y: 30 };
const LIGHT_AMP    = 20;

// Cluster start position (user-settable via "Set Start Pos" button + click)
let CLUSTER_CX   = WORLD_W * 0.08;
let CLUSTER_CY   = WORLD_H * 0.5;
let startPosMode = false;

// Extension 8: Light source 2
let lightSource2    = { x: 35, y: 30 };
let light2On        = false;
let LIGHT_MODE2     = 'static';
let LIGHT_SPEED2    = 0.03;
let lightAngle2     = 0;
const LIGHT_CENTER2 = { x: 35, y: 30 };
let nextLightClick  = 0;   // 0 = next click positions L1, 1 = L2

// Physical Inertia — always on (γ=0.85 default, adjustable via slider)
let FRICTION = 0.85;

// Extension 5: Sensor Noise
let NOISE_ON    = false;
let NOISE_SIGMA = 0.0;

// Extension 6: Heterogeneous module sizes
let HET_ON = false;
const HET_RADII = [0.30, 0.50, 0.75];  // S / M / L
const MAX_R_MOD = 0.75;                 // largest possible radius

// Extension 7: Collision-based module failure + manual kill
let COLLISION_FAIL_ON   = false;
let COLLISION_THRESHOLD = 60;           // cumulative collision hits before death
const COLLISION_RESET_STEPS = 50;       // ~1 s at 50 fps — no collision → counter resets
let totalCollisionDeaths = 0;

// MEI recovery tracking (reset on each kill event)
let recoveryActive   = false;   // currently tracking a recovery
let recoveryMEI      = 0;       // MEI target to beat (value at moment of kill)
let recoveryStep     = 0;       // step number when kill happened
let recoveryTime     = -1;      // steps taken to recover (-1 = not yet)
let stepDeaths       = 0;       // deaths in the current step

// Wall
let wallOn        = false;
let wall2On       = false;          // double-slit (Young's experiment)
const WALL_X      = 70;
const WALL_GAP_Y  = 30;
let   WALL_GAP_HALF = 3.5;
const WALL_GAP_SEP  = 16;          // distance between the two slit centres

// Force parameters
let P_ATTRACT = 8;
let P_LIGHT   = 10;
let P_RANDOM  = 0.2;
let W_MORPH   = 0.2;
let N_INIT    = 80;
const INIT_RADIUS = 6;


// Pre-allocated diffusion buffer (avoids per-frame GC pressure)
let _diffNext = [];

// ============================================================
// Entry point
// ============================================================
window.onload = function () {
    canvas = document.getElementById('Morph-Model');
    ctx    = canvas.getContext('2d');
    PX_PER_UNIT = canvas.width / WORLD_W;

    document.getElementById('buttonRandom').addEventListener('click', randomInit);
    document.getElementById('buttonStart') .addEventListener('click', onStart);
    document.getElementById('buttonReset') .addEventListener('click', resetInit);
    document.getElementById('buttonSpawn') .addEventListener('click', spawnDormant);
    document.getElementById('buttonKill')    .addEventListener('click', killRandom20);
    document.getElementById('buttonSetStart').addEventListener('click', () => {
        startPosMode = !startPosMode;
        const btn = document.getElementById('buttonSetStart');
        btn.classList.toggle('active', startPosMode);
        btn.textContent = startPosMode ? 'Click to Place Start' : 'Set Start Pos';
    });

    canvas.addEventListener('click', canvasClick, false);

    // Live-update listeners
    document.getElementById('chkLight').addEventListener('change', e => {
        lightOn = e.target.checked;
        if (!lightOn) { lightSource.x = LIGHT_CENTER.x; lightSource.y = LIGHT_CENTER.y; lightAngle = 0; }
    });
    document.getElementById('chkLight2').addEventListener('change', e => {
        light2On = e.target.checked;
        nextLightClick = 0;
        document.getElementById('rowLight2').style.display = light2On ? 'flex' : 'none';
    });
    document.getElementById('chkWall') .addEventListener('change', e => {
        wallOn  = e.target.checked;
        if (wallOn) { wall2On = false; document.getElementById('chkWall2').checked = false; }
    });
    document.getElementById('chkWall2').addEventListener('change', e => {
        wall2On = e.target.checked;
        if (wall2On) { wallOn = false; document.getElementById('chkWall').checked = false; }
    });
    document.getElementById('chkNoise').addEventListener('change', e => NOISE_ON = e.target.checked);
    document.getElementById('chkHet')     .addEventListener('change', e => HET_ON            = e.target.checked);
    document.getElementById('chkCollFail').addEventListener('change', e => COLLISION_FAIL_ON = e.target.checked);

    _addRangeListener('rngCollThresh', 'lblCollThresh', v => COLLISION_THRESHOLD = v, 0);

    document.getElementById('selLightMode').addEventListener('change', e => {
        LIGHT_MODE = e.target.value;
        if (LIGHT_MODE === 'static') {
            lightSource.x = LIGHT_CENTER.x; lightSource.y = LIGHT_CENTER.y; lightAngle = 0;
        }
    });
    document.getElementById('selLightMode2').addEventListener('change', e => {
        LIGHT_MODE2 = e.target.value;
        if (LIGHT_MODE2 === 'static') {
            lightSource2.x = LIGHT_CENTER2.x; lightSource2.y = LIGHT_CENTER2.y; lightAngle2 = 0;
        }
    });

    _addRangeListener('rngN',           'lblN',           v => N_INIT       = v, 0);
    _addRangeListener('rngLightSpeed',  'lblLightSpeed',  v => LIGHT_SPEED  = v, 3);
    _addRangeListener('rngLightSpeed2', 'lblLightSpeed2', v => LIGHT_SPEED2 = v, 3);
    _addRangeListener('rngNoiseSigma',  'lblNoiseSigma',  v => NOISE_SIGMA  = v, 2);
    _addRangeListener('rngFriction',    'lblFriction',    v => FRICTION      = v, 2);

    initFromUI();
    resetInit();
};

function _addRangeListener(rangeId, labelId, setter, decimals) {
    const el = document.getElementById(rangeId);
    const lb = document.getElementById(labelId);
    el.addEventListener('input', e => {
        const v = parseFloat(e.target.value);
        setter(v);
        if (lb) lb.textContent = v.toFixed(decimals);
    });
    if (lb) lb.textContent = parseFloat(el.value).toFixed(decimals);
}

function initFromUI() {
    N_INIT     = parseInt(document.getElementById('rngN').value, 10);
    P_ATTRACT  = parseFloat(document.getElementById('inpP').value);
    P_LIGHT    = parseFloat(document.getElementById('inpL').value);
    P_RANDOM   = parseFloat(document.getElementById('inpR').value);
    W_MORPH    = parseFloat(document.getElementById('inpW').value);
    WALL_GAP_HALF = parseFloat(document.getElementById('inpGap').value) / 2;
    lightOn  = document.getElementById('chkLight').checked;
    wallOn   = document.getElementById('chkWall').checked;
    wall2On  = document.getElementById('chkWall2').checked;
    NOISE_ON = document.getElementById('chkNoise').checked;
    HET_ON            = document.getElementById('chkHet').checked;
    COLLISION_FAIL_ON = document.getElementById('chkCollFail').checked;
    COLLISION_THRESHOLD = parseFloat(document.getElementById('rngCollThresh').value);
    LIGHT_MODE  = document.getElementById('selLightMode').value;
    LIGHT_MODE2 = document.getElementById('selLightMode2').value;
    LIGHT_SPEED  = parseFloat(document.getElementById('rngLightSpeed').value);
    LIGHT_SPEED2 = parseFloat(document.getElementById('rngLightSpeed2').value);
    light2On     = document.getElementById('chkLight2').checked;
    NOISE_SIGMA  = parseFloat(document.getElementById('rngNoiseSigma').value);
    FRICTION     = parseFloat(document.getElementById('rngFriction').value);
}

// ============================================================
// Start / Stop
// ============================================================
function onStart() {
    initFromUI();
    if (running) {
        clearInterval(timer);
        document.getElementById('buttonStart').textContent = 'Start';
        running = false;
    } else {
        timer = setInterval(step, 20);
        document.getElementById('buttonStart').textContent = 'Stop';
        running = true;
    }
}

// ============================================================
// Initialization
// ============================================================
function resetInit() {
    if (running) { clearInterval(timer); running = false; document.getElementById('buttonStart').textContent = 'Start'; }
    initFromUI();
    modules    = [];
    iterate     = 0;
    lightAngle  = 0;
    lightAngle2 = 0;
    totalCollisionDeaths = 0;
    nextLightClick = 0;
    recoveryActive = false; recoveryMEI = 0; recoveryStep = 0; recoveryTime = -1;
    lightSource = { x: LIGHT_CENTER.x, y: LIGHT_CENTER.y };
    pMinEMA = 0; pMaxEMA = 1;

    const cx = CLUSTER_CX;
    const cy = CLUSTER_CY;
    let tries = 0;
    while (modules.length < N_INIT && tries < N_INIT * 400) {
        const newR = _pickRadius();
        const r  = Math.sqrt(Math.random()) * INIT_RADIUS;
        const th = Math.random() * 2 * Math.PI;
        const x  = cx + r * Math.cos(th);
        const y  = cy + r * Math.sin(th);
        if (_noOverlap(x, y, newR)) modules.push(newModule(x, y, newR));
        tries++;
    }
    render(); updateStats(computeMEI());
}

function randomInit() {
    if (running) { clearInterval(timer); running = false; document.getElementById('buttonStart').textContent = 'Start'; }
    initFromUI();
    modules = []; iterate = 0;
    lightAngle = 0; lightAngle2 = 0;
    totalCollisionDeaths = 0;
    nextLightClick = 0;
    recoveryActive = false; recoveryMEI = 0; recoveryStep = 0; recoveryTime = -1;
    let tries = 0;
    while (modules.length < N_INIT && tries < N_INIT * 400) {
        const newR = _pickRadius();
        const x = Math.random() * WORLD_W;
        const y = Math.random() * WORLD_H;
        if (_noOverlap(x, y, newR)) modules.push(newModule(x, y, newR));
        tries++;
    }
    render(); updateStats(computeMEI());
}

function spawnDormant() {
    const n = Math.max(10, Math.floor(N_INIT / 2));
    let added = 0, tries = 0;
    while (added < n && tries < n * 400) {
        const newR = _pickRadius();
        const x = Math.random() * WORLD_W;
        const y = Math.random() * WORLD_H;
        if (_noOverlap(x, y, newR)) {
            const m = newModule(x, y, newR);
            m.active = false;
            modules.push(m);
            added++;
        }
        tries++;
    }
    render(); updateStats(computeMEI());
}

// Kill a random 20% of active modules instantly (manual failure event).
// Records pre-kill MEI as recovery target.
function killRandom20() {
    const meiBefore = computeMEI();   // target: recover to this level
    const active = modules.filter(m => m.active);
    const killN  = Math.max(1, Math.floor(active.length * 0.2));
    // Fisher-Yates partial shuffle to pick killN victims
    for (let i = 0; i < killN; i++) {
        const j = i + Math.floor(Math.random() * (active.length - i));
        [active[i], active[j]] = [active[j], active[i]];
        active[i].active = false;
        active[i].vx = 0; active[i].vy = 0;
        active[i].collisions = 0;
        totalCollisionDeaths++;
        stepDeaths++;
    }
    recoveryActive = true;
    recoveryMEI    = meiBefore;
    recoveryStep   = iterate;
    recoveryTime   = -1;
    render(); updateStats(computeMEI());
}

function _pickRadius() {
    if (!HET_ON) return R_MOD;
    const r = Math.random();
    return r < 0.40 ? HET_RADII[0] : r < 0.80 ? HET_RADII[1] : HET_RADII[2];
}

function newModule(x, y, radius = R_MOD) {
    return { x, y, vx: 0, vy: 0, active: true, inShadow: false, inShadow2: false, radius,
             collisions: 0, lastCollisionStep: 0,
             tokens: new Float64Array(TOKEN_MAX + 2), potential: 0 };
}

function _noOverlap(x, y, newR) {
    for (const m of modules) {
        const minDist = m.radius + newR;
        if ((m.x - x) * (m.x - x) + (m.y - y) * (m.y - y) < minDist * minDist) return false;
    }
    return true;
}

// ============================================================
// Utilities
// ============================================================
function dist2(x1, y1, x2, y2) { return (x1 - x2) ** 2 + (y1 - y2) ** 2; }

// Box-Muller Gaussian noise (Extension 5)
function gaussianNoise(sigma) {
    const u1 = Math.random() + 1e-12;
    const u2 = Math.random();
    return sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

// ============================================================
// Spatial hash
// ============================================================
function rebuildHash() {
    spatialHash = new Map();
    for (let i = 0; i < modules.length; i++) {
        const key = _hkey(modules[i].x, modules[i].y);
        let b = spatialHash.get(key);
        if (!b) { b = []; spatialHash.set(key, b); }
        b.push(i);
    }
}

function _hkey(x, y) {
    return (Math.floor(x / HASH_CELL) * 100003 + Math.floor(y / HASH_CELL)) | 0;
}

function neighborsWithin(mi, r) {
    const m  = modules[mi];
    const cx = Math.floor(m.x / HASH_CELL);
    const cy = Math.floor(m.y / HASH_CELL);
    const r2 = r * r;
    const out = [];
    for (let dx = -1; dx <= 1; dx++) {
        for (let dy = -1; dy <= 1; dy++) {
            const b = spatialHash.get((cx + dx) * 100003 + (cy + dy));
            if (!b) continue;
            for (const j of b) {
                if (j === mi) continue;
                if (dist2(modules[j].x, modules[j].y, m.x, m.y) <= r2) out.push(j);
            }
        }
    }
    return out;
}

// ============================================================
// Token diffusion — continuous-space port of Ishida num_diffuse()
// Extension 5: optional Gaussian noise on token counts
// ============================================================
// Phase-2 correction: distribute each module's tokens to ALL active neighbors
// every round (Ishida: tokens spread equally to neighbors, value +1 per hop,
// minus a residual rate), instead of one random neighbor. The single-neighbor
// scheme made the per-step potential very noisy and broke potential cohesion;
// all-neighbor exchange yields a smooth, center-high gradient.
function diffuseTokens() {
    const N = modules.length;

    // Seed: each active module holds 1 token of value 1
    for (let i = 0; i < N; i++) {
        modules[i].tokens.fill(0);
        if (modules[i].active) modules[i].tokens[1] = 1;
    }

    if (_diffNext.length !== N) {
        _diffNext = Array.from({ length: N }, () => new Float64Array(TOKEN_MAX + 2));
    }

    // positions are fixed during diffusion -> compute active neighbor lists once
    const neighLists = new Array(N);
    for (let i = 0; i < N; i++) {
        if (!modules[i].active) { neighLists[i] = null; continue; }
        neighLists[i] = neighborsWithin(i, R_NEIGH).filter(j => modules[j].active);
    }

    for (let s = 0; s < DIFFUSE_STEPS; s++) {
        for (let i = 0; i < N; i++) _diffNext[i].fill(0);

        for (let i = 0; i < N; i++) {
            if (!modules[i].active) continue;
            const neigh = neighLists[i];
            const deg = neigh.length;
            for (let n = 1; n <= TOKEN_MAX - 1; n++) {
                let t = modules[i].tokens[n];
                if (t === 0) continue;
                // Extension 5: Gaussian noise on token exchange
                if (NOISE_ON && NOISE_SIGMA > 0) {
                    t = Math.max(0, t + gaussianNoise(NOISE_SIGMA));
                }
                _diffNext[i][n] += t * DIFFUSE_RESID;
                if (deg > 0) {
                    const share = t * (1 - DIFFUSE_RESID) / deg;
                    for (const j of neigh) _diffNext[j][n + 1] += share;
                } else {
                    _diffNext[i][n] += t * (1 - DIFFUSE_RESID);
                }
            }
        }

        for (let i = 0; i < N; i++) modules[i].tokens.set(_diffNext[i]);
    }
}

// ============================================================
// Potential computation — Ishida eq.(2): Potential = N1 - w*N2,
//   N1 = count of ALL tokens (1..2L), N2 = count of INNER tokens (1..L).
//   (The earlier port had inner - w*all, which inverted the gradient.)
// EMA-smoothed.
// ============================================================
function computePotentials() {
    const alpha = 0.3;
    for (const m of modules) {
        let all = 0, inner = 0;
        for (let n = 1; n <= TOKEN_MAX; n++) {
            const t = m.tokens[n];
            all += t;
            if (n <= SUM2_LIMIT) inner += t;
        }
        const raw = all - inner * W_MORPH;
        m.potential = (1 - alpha) * m.potential + alpha * raw;
    }
}

// ============================================================
// Dormancy transitions — paper Section 2.3, steps 7 & 8
//
// Active → dormant: omitted in continuous model.
//   In the discrete version, the dense grid potential is stable enough
//   for a clean threshold at 0.  In continuous space with ~80 sparse
//   particles the per-step potential is too noisy; using the same
//   threshold instantly kills the whole swarm on step 1.
//   The effect is preserved indirectly: spawned dormant modules only
//   wake up inside the high-potential core of the cluster.
//
// Dormant → active: wake up when an active neighbor (potential ≥ 0)
//   is within R_NEIGH.  This matches paper step 8.
// ============================================================
function updateDormancy() {
    for (let i = 0; i < modules.length; i++) {
        const m = modules[i];
        if (!m.active) {
            for (const j of neighborsWithin(i, R_NEIGH)) {
                if (modules[j].active && modules[j].potential >= 0) {
                    m.active = true;
                    break;
                }
            }
        }
    }
}

// ============================================================
// Extension 4: Dynamic Light Source update
// ============================================================
function updateLightSource() {
    if (lightOn && LIGHT_MODE !== 'static') {
        if (LIGHT_MODE === 'oscillate') {
            lightSource.x = LIGHT_CENTER.x + Math.sin(lightAngle) * LIGHT_AMP;
            lightSource.y = LIGHT_CENTER.y;
        } else {
            lightSource.x = LIGHT_CENTER.x + Math.cos(lightAngle) * LIGHT_AMP;
            lightSource.y = LIGHT_CENTER.y + Math.sin(lightAngle) * LIGHT_AMP;
        }
        lightAngle += LIGHT_SPEED;
    }
    if (light2On && LIGHT_MODE2 !== 'static') {
        if (LIGHT_MODE2 === 'oscillate') {
            lightSource2.x = LIGHT_CENTER2.x + Math.sin(lightAngle2) * LIGHT_AMP;
            lightSource2.y = LIGHT_CENTER2.y;
        } else {
            lightSource2.x = LIGHT_CENTER2.x + Math.cos(lightAngle2) * LIGHT_AMP;
            lightSource2.y = LIGHT_CENTER2.y + Math.sin(lightAngle2) * LIGHT_AMP;
        }
        lightAngle2 += LIGHT_SPEED2;
    }
}

// ============================================================
// Shadow computation (paper Section 2.3 step 3 — light sensing)
//
// For each active module i, cast a ray from the light source to i.
// If any other active module j lies on that ray (within R_MOD of the
// ray, and closer to the light), module i is in shadow and receives
// no light force this step.
//
// The discrete code computed castShadow() but only zeroed intensity,
// not direction, so it never actually blocked light forces on modules.
// Here we connect shadow detection directly to force application.
// ============================================================
function _computeShadowForLight(lx, ly, prop) {
    const N = modules.length;
    for (let i = 0; i < N; i++) modules[i][prop] = false;
    for (let i = 0; i < N; i++) {
        const mi = modules[i];
        if (!mi.active) continue;
        const rix = mi.x - lx, riy = mi.y - ly;
        const dist_i2 = rix * rix + riy * riy;
        if (dist_i2 < 1e-10) continue;
        for (let j = 0; j < N; j++) {
            if (j === i) continue;
            const mj = modules[j];
            if (!mj.active) continue;
            const rjx = mj.x - lx, rjy = mj.y - ly;
            const t = (rjx * rix + rjy * riy) / dist_i2;
            if (t <= 0 || t >= 1) continue;
            const px = rjx - t * rix, py = rjy - t * riy;
            if (px * px + py * py < mj.radius * mj.radius) { mi[prop] = true; break; }
        }
    }
}

function computeShadows() {
    if (lightOn)  _computeShadowForLight(lightSource.x,  lightSource.y,  'inShadow');
    else          for (const m of modules) m.inShadow  = false;
    if (light2On) _computeShadowForLight(lightSource2.x, lightSource2.y, 'inShadow2');
    else          for (const m of modules) m.inShadow2 = false;
}

// ============================================================
// Force calculation + movement
// Extension 2: INERTIA_ON switches between overdamped and inertial dynamics
// Extension 5: noise on perceived light direction
// ============================================================
function applyForces() {
    for (let i = 0; i < modules.length; i++) {
        const m = modules[i];

        if (!m.active) {
            // Dormant: random walk at meaningful speed (paper step 7).
            // Speed target ≈ 40 % of MAX_STEP so dormant modules visibly
            // drift and can eventually reach the active cluster.
            const angle  = Math.random() * 2 * Math.PI;
            const dspd   = MAX_STEP * 0.4;
            m.vx = m.vx * FRICTION + Math.cos(angle) * dspd * (1 - FRICTION);
            m.vy = m.vy * FRICTION + Math.sin(angle) * dspd * (1 - FRICTION);
            continue;
        }

        // Collision counter reset: no collision for ~1 s → healed
        if (iterate - m.lastCollisionStep >= COLLISION_RESET_STEPS) m.collisions = 0;

        let fx = 0, fy = 0;

        // --- (A) Overlap repulsion ---
        const touchSearch = m.radius + (HET_ON ? MAX_R_MOD : R_MOD);
        for (const j of neighborsWithin(i, touchSearch)) {
            const n = modules[j];
            const TOUCH_IJ = m.radius + n.radius;
            const dx = n.x - m.x, dy = n.y - m.y;
            const d  = Math.hypot(dx, dy) + 1e-6;
            if (d >= TOUCH_IJ) continue;
            m.collisions++;
            m.lastCollisionStep = iterate;
            const push = (d - TOUCH_IJ) * 8;
            fx += push * dx / d;
            fy += push * dy / d;
        }

        // Death by collision fatigue
        if (COLLISION_FAIL_ON && m.collisions >= COLLISION_THRESHOLD) {
            m.active = false;
            m.collisions = 0;
            m.vx = 0; m.vy = 0;
            totalCollisionDeaths++;
            stepDeaths++;
            continue;
        }

        // --- (A) Potential-gradient attraction — paper's A force ---
        const pSelf = Math.abs(m.potential) + 1;
        for (const j of neighborsWithin(i, R_NEIGH)) {
            const n = modules[j];
            if (!n.active) continue;
            const dx = n.x - m.x, dy = n.y - m.y;
            const d  = Math.hypot(dx, dy) + 1e-6;
            const ratio = (n.potential - m.potential) / pSelf;
            const s = ratio * P_ATTRACT * 0.025;
            fx += s * dx / d;
            fy += s * dy / d;
        }

        // --- (L) Light source force — paper's L force ---
        // Shadow: front modules (inShadow=false) get full force.
        // Interior modules in shadow get 25% — enough to keep the cluster
        // moving as a unit while still biasing movement toward the front face.
        if (lightOn) {
            const shadowFactor = m.inShadow ? 0.40 : 1.0;
            let ldx = lightSource.x - m.x;
            let ldy = lightSource.y - m.y;
            // Extension 5: Gaussian noise on perceived light direction
            if (NOISE_ON && NOISE_SIGMA > 0) {
                const angle = Math.atan2(ldy, ldx) + gaussianNoise(NOISE_SIGMA * 0.3);
                ldx = Math.cos(angle);
                ldy = Math.sin(angle);
            } else {
                const ld = Math.hypot(ldx, ldy) + 1e-6;
                ldx /= ld; ldy /= ld;
            }
            fx += ldx * P_LIGHT * 0.08 * shadowFactor;
            fy += ldy * P_LIGHT * 0.08 * shadowFactor;
        }

        // --- (L2) Light source 2 force (Extension 8) ---
        if (light2On) {
            const shadowFactor2 = m.inShadow2 ? 0.40 : 1.0;
            let l2dx = lightSource2.x - m.x;
            let l2dy = lightSource2.y - m.y;
            if (NOISE_ON && NOISE_SIGMA > 0) {
                const angle = Math.atan2(l2dy, l2dx) + gaussianNoise(NOISE_SIGMA * 0.3);
                l2dx = Math.cos(angle); l2dy = Math.sin(angle);
            } else {
                const ld = Math.hypot(l2dx, l2dy) + 1e-6;
                l2dx /= ld; l2dy /= ld;
            }
            fx += l2dx * P_LIGHT * 0.08 * shadowFactor2;
            fy += l2dy * P_LIGHT * 0.08 * shadowFactor2;
        }

        // --- (R) Random force — paper's R force ---
        // Scaled to ~10% of light force magnitude, matching discrete r=1 ratio.
        fx += (Math.random() * 2 - 1) * P_RANDOM * 0.20;
        fy += (Math.random() * 2 - 1) * P_RANDOM * 0.20;

        // Physical inertia: v += F·dt with friction, speed-capped at MAX_STEP.
        m.vx = m.vx * FRICTION + fx * DT;
        m.vy = m.vy * FRICTION + fy * DT;
        const spd = Math.hypot(m.vx, m.vy);
        if (spd > MAX_STEP) { m.vx *= MAX_STEP / spd; m.vy *= MAX_STEP / spd; }
    }

    // Apply displacements with wall + world-boundary enforcement
    for (const m of modules) {
        let nx = m.x + m.vx;
        let ny = m.y + m.vy;

        if (wallOn || wall2On) {
            const crossing = (m.x - WALL_X) * (nx - WALL_X) < 0;
            if (crossing) {
                const t = (WALL_X - m.x) / (nx - m.x + 1e-9);
                const yAtWall = m.y + t * (ny - m.y);
                let blocked;
                if (wallOn) {
                    blocked = Math.abs(yAtWall - WALL_GAP_Y) > WALL_GAP_HALF;
                } else {
                    // Double-slit: pass through either of the two gaps
                    const inGap1 = Math.abs(yAtWall - (WALL_GAP_Y - WALL_GAP_SEP / 2)) <= WALL_GAP_HALF;
                    const inGap2 = Math.abs(yAtWall - (WALL_GAP_Y + WALL_GAP_SEP / 2)) <= WALL_GAP_HALF;
                    blocked = !inGap1 && !inGap2;
                }
                if (blocked) {
                    nx = (m.x < WALL_X) ? WALL_X - R_MOD : WALL_X + R_MOD;
                    m.vx = -m.vx * 0.3;
                }
            }
        }

        m.x = Math.max(R_MOD, Math.min(WORLD_W - R_MOD, nx));
        m.y = Math.max(R_MOD, Math.min(WORLD_H - R_MOD, ny));
    }
}

// ============================================================
// MEI — Morphological Evaluation Index (paper Eq. 4)
// FIX: counts active modules only (original bug: included dormant)
// ============================================================
function computeMEI() {
    const act = modules.filter(m => m.active);
    if (act.length < 2) return 1;
    let cx = 0, cy = 0;
    for (const m of act) { cx += m.x; cy += m.y; }
    cx /= act.length; cy /= act.length;
    let s1 = 0;
    for (const m of act) s1 += Math.hypot(m.x - cx, m.y - cy);
    // s2: expected sum for same N modules in a perfect circle, hex-packed
    const avgR  = act.reduce((s, m) => s + m.radius, 0) / act.length;
    // Ideal hex-packed disc radius: pi*Rpack^2 = N*pi*r^2/phi -> Rpack = r*sqrt(N/phi),
    // phi = 0.907. (Previously divided by an extra pi, so a perfect circle read ~1.77
    // instead of 1.0; corrected for an Ishida-comparable scale.)
    const packR = avgR * Math.sqrt(act.length / 0.907);
    const s2 = act.length * (2 * packR / 3);
    return s1 / s2;
}

// ============================================================
// Main step
// ============================================================
function step() {
    iterate++;
    stepDeaths = 0;
    rebuildHash();
    diffuseTokens();
    computePotentials();
    updateDormancy();
    updateLightSource();
    computeShadows();
    applyForces();
    render();
    const mei = computeMEI();
    // MEI recovery check: did we recover to the pre-kill MEI?
    if (recoveryActive && recoveryTime === -1 && mei <= recoveryMEI) {
        recoveryTime   = iterate - recoveryStep;
        recoveryActive = false;
    }
    updateStats(mei);
}


function updateStats(mei) {
    let nA = 0, nD = 0;
    for (const m of modules) {
        if (m.active) nA++; else nD++;
    }
    let txt = `Step: ${iterate} | Active: ${nA} | Dormant: ${nD} | MEI: ${mei.toFixed(3)}`;
    if (lightOn && LIGHT_MODE !== 'static') {
        const lx = lightSource.x.toFixed(1), ly = lightSource.y.toFixed(1);
        txt += ` | L1:(${lx},${ly})`;
    }
    if (light2On && LIGHT_MODE2 !== 'static') {
        const lx = lightSource2.x.toFixed(1), ly = lightSource2.y.toFixed(1);
        txt += ` | L2:(${lx},${ly})`;
    }
    if (light2On) txt += ` | Click→L${nextLightClick + 1}`;
    if (NOISE_ON) txt += ` | Noise σ=${NOISE_SIGMA.toFixed(2)}`;
    if (HET_ON) {
        let nS = 0, nM = 0, nL = 0;
        for (const m of modules) {
            if (!m.active) continue;
            if (m.radius <= 0.35) nS++; else if (m.radius <= 0.60) nM++; else nL++;
        }
        txt += ` | S:${nS} M:${nM} L:${nL}`;
    }
    if (COLLISION_FAIL_ON || totalCollisionDeaths > 0)
        txt += ` | Deaths:${totalCollisionDeaths}`;
    if (recoveryActive)
        txt += ` | Recovering… (${iterate - recoveryStep} steps)`;
    else if (recoveryTime >= 0)
        txt += ` | Recovered in ${recoveryTime} steps`;
    document.getElementById('stats').textContent = txt;
}

// ============================================================
// Rendering
// ============================================================
function render() {
    ctx.fillStyle = '#0a1628';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Light source 1 glow (yellow)
    if (lightOn) {
        const lx = lightSource.x * PX_PER_UNIT;
        const ly = lightSource.y * PX_PER_UNIT;
        const grad = ctx.createRadialGradient(lx, ly, 0, lx, ly, 70);
        grad.addColorStop(0, 'rgba(255,220,80,0.85)');
        grad.addColorStop(1, 'rgba(255,220,80,0)');
        ctx.fillStyle = grad;
        ctx.beginPath(); ctx.arc(lx, ly, 70, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = '#ffe040';
        ctx.beginPath(); ctx.arc(lx, ly, 4, 0, 2 * Math.PI); ctx.fill();
    }

    // Light source 2 glow (cyan)
    if (light2On) {
        const lx = lightSource2.x * PX_PER_UNIT;
        const ly = lightSource2.y * PX_PER_UNIT;
        const grad = ctx.createRadialGradient(lx, ly, 0, lx, ly, 70);
        grad.addColorStop(0, 'rgba(80,220,255,0.80)');
        grad.addColorStop(1, 'rgba(80,220,255,0)');
        ctx.fillStyle = grad;
        ctx.beginPath(); ctx.arc(lx, ly, 70, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = '#40e0ff';
        ctx.beginPath(); ctx.arc(lx, ly, 4, 0, 2 * Math.PI); ctx.fill();
    }

    // Wall rendering
    if (wallOn || wall2On) {
        const wx = WALL_X * PX_PER_UNIT;
        ctx.fillStyle = '#2a7a50';
        if (wallOn) {
            // Single slit: two segments
            const gy1 = (WALL_GAP_Y - WALL_GAP_HALF) * PX_PER_UNIT;
            const gy2 = (WALL_GAP_Y + WALL_GAP_HALF) * PX_PER_UNIT;
            ctx.fillRect(wx - 4, 0, 8, gy1);
            ctx.fillRect(wx - 4, gy2, 8, canvas.height - gy2);
        } else {
            // Double slit: three segments
            const s1top = (WALL_GAP_Y - WALL_GAP_SEP / 2 - WALL_GAP_HALF) * PX_PER_UNIT;
            const s1bot = (WALL_GAP_Y - WALL_GAP_SEP / 2 + WALL_GAP_HALF) * PX_PER_UNIT;
            const s2top = (WALL_GAP_Y + WALL_GAP_SEP / 2 - WALL_GAP_HALF) * PX_PER_UNIT;
            const s2bot = (WALL_GAP_Y + WALL_GAP_SEP / 2 + WALL_GAP_HALF) * PX_PER_UNIT;
            ctx.fillRect(wx - 4, 0,     8, s1top);                  // above slit 1
            ctx.fillRect(wx - 4, s1bot, 8, s2top - s1bot);          // between slits
            ctx.fillRect(wx - 4, s2bot, 8, canvas.height - s2bot);  // below slit 2
        }
    }

    // Module color scale (EMA to prevent flicker)
    let pMin = Infinity, pMax = -Infinity;
    for (const m of modules) {
        if (m.potential < pMin) pMin = m.potential;
        if (m.potential > pMax) pMax = m.potential;
    }
    pMinEMA = 0.9 * pMinEMA + 0.1 * (isFinite(pMin) ? pMin : 0);
    pMaxEMA = 0.9 * pMaxEMA + 0.1 * (isFinite(pMax) ? pMax : 1);
    const pRange = Math.max(pMaxEMA - pMinEMA, 1e-6);

    for (const m of modules) {
        const t = Math.max(0, Math.min(1, (m.potential - pMinEMA) / pRange));
        ctx.fillStyle = potentialColor(t, m.active);
        ctx.beginPath();
        ctx.arc(m.x * PX_PER_UNIT, m.y * PX_PER_UNIT, m.radius * PX_PER_UNIT, 0, 2 * Math.PI);
        ctx.fill();
    }

    // Start position marker — only shown while Set Start Pos mode is active
    if (startPosMode) {
        const mx = CLUSTER_CX * PX_PER_UNIT;
        const my = CLUSTER_CY * PX_PER_UNIT;
        const mr = INIT_RADIUS * PX_PER_UNIT;
        ctx.strokeStyle = '#a78bfa';
        ctx.lineWidth   = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.arc(mx, my, mr, 0, 2 * Math.PI); ctx.stroke();
        ctx.setLineDash([]);
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(mx - 6, my); ctx.lineTo(mx + 6, my); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(mx, my - 6); ctx.lineTo(mx, my + 6); ctx.stroke();
    }

    // Step counter overlay
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.fillRect(5, 5, 80, 22);
    ctx.fillStyle = '#fff';
    ctx.font = '14px monospace';
    ctx.fillText('t=' + iterate, 10, 21);
}

function potentialColor(t, active) {
    if (!active) return 'rgb(80,80,100)';
    if (t < 0.5) {
        const k = t * 2;
        return `rgb(${(50 * k) | 0},${(140 + 60 * k) | 0},${(200 - 150 * k) | 0})`;
    }
    const k = (t - 0.5) * 2;
    return `rgb(${(50 + 150 * k) | 0},${(200 - 100 * k) | 0},${(50 - 50 * k) | 0})`;
}


// ============================================================
// Canvas click — sequential light placement (E8)
// Single light: click sets light 1 (static mode only, as before).
// Two lights on: clicks alternate L1 → L2 → L1 → …
// Clicking also updates the orbit center so dynamic mode follows.
// ============================================================
function canvasClick(e) {
    const rect = canvas.getBoundingClientRect();
    const wx = (e.clientX - rect.left) / PX_PER_UNIT;
    const wy = (e.clientY - rect.top)  / PX_PER_UNIT;

    // Start position mode: click places the cluster marker
    if (startPosMode) {
        CLUSTER_CX = Math.max(INIT_RADIUS + 1, Math.min(WORLD_W - INIT_RADIUS - 1, wx));
        CLUSTER_CY = Math.max(INIT_RADIUS + 1, Math.min(WORLD_H - INIT_RADIUS - 1, wy));
        render();
        return;
    }

    if (!light2On) {
        if (!lightOn || LIGHT_MODE !== 'static') return;
        lightSource.x = wx; lightSource.y = wy;
        LIGHT_CENTER.x = wx; LIGHT_CENTER.y = wy;
    } else {
        if (nextLightClick === 0) {
            lightSource.x = wx; lightSource.y = wy;
            LIGHT_CENTER.x = wx; LIGHT_CENTER.y = wy;
            nextLightClick = 1;
        } else {
            lightSource2.x = wx; lightSource2.y = wy;
            LIGHT_CENTER2.x = wx; LIGHT_CENTER2.y = wy;
            nextLightClick = 0;
        }
    }
    render();
}
