//
// sim_core.js
// Headless, DOM-free port of cell_robot_continuous.js for batch experiments.
//
// The physics is a faithful copy of the interactive browser implementation
// (Ishida-2025 continuous-space extension, E1-E8). The only changes are:
//   - all DOM / canvas / rendering code removed
//   - Math.random replaced by a seeded PRNG (mulberry32) for reproducibility
//   - all parameters injected through a config object
//   - the simulation is wrapped in a class so many runs are independent
//
// Exports: { Sim, mulberry32, DEFAULTS }
//

'use strict';

// ---- Seeded PRNG (mulberry32) ----------------------------------------------
function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

// ---- Defaults (mirror the browser constants / standard parameters) ---------
const DEFAULTS = {
    WORLD_W: 140,
    WORLD_H: 60,
    R_MOD: 0.5,
    R_NEIGH: 1.8,
    DT: 0.12,
    MAX_STEP: 0.14,
    TOKEN_MAX: 20,
    DIFFUSE_STEPS: 20,
    DIFFUSE_RESID: 0.3,   // residual rate: fraction of tokens kept per round (Ishida)
    SUM2_LIMIT: 10,
    INIT_RADIUS: 6,

    FRICTION: 0.85,
    P_ATTRACT: 8,
    P_LIGHT: 10,
    P_RANDOM: 0.2,
    W_MORPH: 0.2,
    N_INIT: 80,

    HET_RADII: [0.30, 0.50, 0.75],
    MAX_R_MOD: 0.75,

    COLLISION_THRESHOLD: 60,
    COLLISION_RESET_STEPS: 50,

    WALL_X: 70,
    WALL_GAP_Y: 30,
    WALL_GAP: 7,          // full gap width (HALF = gap/2)
    WALL_GAP_SEP: 16,

    LIGHT_AMP: 20,
    EMA_ALPHA: 0.3,

    // cluster start (paper baseline: x = 140*0.08 = 11.2, y = 30)
    CLUSTER_CX: 140 * 0.08,
    CLUSTER_CY: 30,

    // light 1
    LIGHT_X: 105,
    LIGHT_Y: 30,
    LIGHT_ON: true,
    LIGHT_MODE: 'static',   // 'static' | 'oscillate' | 'circular'
    LIGHT_SPEED: 0.03,

    // light 2 (E8)
    LIGHT2_X: 35,
    LIGHT2_Y: 30,
    LIGHT2_ON: false,
    LIGHT2_MODE: 'static',
    LIGHT2_SPEED: 0.03,

    // scenario toggles
    WALL_ON: false,
    WALL2_ON: false,
    NOISE_ON: false,
    NOISE_SIGMA: 0.0,
    HET_ON: false,
    COLLISION_FAIL_ON: false,

    seed: 12345,
};

class Sim {
    constructor(cfg = {}) {
        this.p = Object.assign({}, DEFAULTS, cfg);
        this._rng = mulberry32(this.p.seed >>> 0);
        this.HASH_CELL = this.p.R_NEIGH;

        this.modules = [];
        this.iterate = 0;
        this.spatialHash = null;
        this._diffNext = [];

        this.lightSource  = { x: this.p.LIGHT_X,  y: this.p.LIGHT_Y };
        this.lightCenter  = { x: this.p.LIGHT_X,  y: this.p.LIGHT_Y };
        this.lightAngle   = 0;
        this.lightSource2 = { x: this.p.LIGHT2_X, y: this.p.LIGHT2_Y };
        this.lightCenter2 = { x: this.p.LIGHT2_X, y: this.p.LIGHT2_Y };
        this.lightAngle2  = 0;

        this.totalCollisionDeaths = 0;
        this.stepDeaths = 0;

        // MEI recovery tracking
        this.recoveryActive = false;
        this.recoveryMEI = 0;
        this.recoveryStep = 0;
        this.recoveryTime = -1;
    }

    random() { return this._rng(); }

    // ---- init ----
    _pickRadius() {
        if (!this.p.HET_ON) return this.p.R_MOD;
        const r = this.random();
        const H = this.p.HET_RADII;
        return r < 0.40 ? H[0] : r < 0.80 ? H[1] : H[2];
    }

    _newModule(x, y, radius) {
        return { x, y, vx: 0, vy: 0, active: true, inShadow: false, inShadow2: false,
                 radius, collisions: 0, lastCollisionStep: 0,
                 tokens: new Float64Array(this.p.TOKEN_MAX + 2), potential: 0 };
    }

    _noOverlap(x, y, newR) {
        for (const m of this.modules) {
            const minDist = m.radius + newR;
            if ((m.x - x) ** 2 + (m.y - y) ** 2 < minDist * minDist) return false;
        }
        return true;
    }

    // cluster init around (CLUSTER_CX, CLUSTER_CY)
    initCluster() {
        this.modules = [];
        const cx = this.p.CLUSTER_CX, cy = this.p.CLUSTER_CY;
        let tries = 0;
        while (this.modules.length < this.p.N_INIT && tries < this.p.N_INIT * 400) {
            const newR = this._pickRadius();
            const r  = Math.sqrt(this.random()) * this.p.INIT_RADIUS;
            const th = this.random() * 2 * Math.PI;
            const x  = cx + r * Math.cos(th);
            const y  = cy + r * Math.sin(th);
            if (this._noOverlap(x, y, newR)) this.modules.push(this._newModule(x, y, newR));
            tries++;
        }
    }

    // scatter active modules randomly across the whole world
    initScatter(n = this.p.N_INIT) {
        this.modules = [];
        let tries = 0;
        while (this.modules.length < n && tries < n * 400) {
            const newR = this._pickRadius();
            const x = this.random() * this.p.WORLD_W;
            const y = this.random() * this.p.WORLD_H;
            if (this._noOverlap(x, y, newR)) this.modules.push(this._newModule(x, y, newR));
            tries++;
        }
    }

    // place dormant modules across the whole world at given probability density
    // (approximates the browser "+ Dormant" / Ishida prerandom placement).
    // prob ~ fraction of free space to fill; we sample candidate points on a grid.
    spawnDormantProb(prob) {
        const step = this.p.R_MOD * 2.2;
        for (let x = step; x < this.p.WORLD_W - step; x += step) {
            for (let y = step; y < this.p.WORLD_H - step; y += step) {
                if (this.random() >= prob) continue;
                const newR = this._pickRadius();
                if (this._noOverlap(x, y, newR)) {
                    const m = this._newModule(x, y, newR);
                    m.active = false;
                    this.modules.push(m);
                }
            }
        }
    }

    spawnDormantCount(n) {
        let added = 0, tries = 0;
        while (added < n && tries < n * 400) {
            const newR = this._pickRadius();
            const x = this.random() * this.p.WORLD_W;
            const y = this.random() * this.p.WORLD_H;
            if (this._noOverlap(x, y, newR)) {
                const m = this._newModule(x, y, newR);
                m.active = false;
                this.modules.push(m);
                added++;
            }
            tries++;
        }
    }

    killRandom20() {
        const meiBefore = this.computeMEI();
        const active = this.modules.filter(m => m.active);
        const killN = Math.max(1, Math.floor(active.length * 0.2));
        for (let i = 0; i < killN; i++) {
            const j = i + Math.floor(this.random() * (active.length - i));
            [active[i], active[j]] = [active[j], active[i]];
            active[i].active = false;
            active[i].vx = 0; active[i].vy = 0;
            active[i].collisions = 0;
            this.totalCollisionDeaths++;
            this.stepDeaths++;
        }
        this.recoveryActive = true;
        this.recoveryMEI = meiBefore;
        this.recoveryStep = this.iterate;
        this.recoveryTime = -1;
        return killN;
    }

    // ---- utilities ----
    _gaussianNoise(sigma) {
        const u1 = this.random() + 1e-12;
        const u2 = this.random();
        return sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }

    // ---- spatial hash ----
    _rebuildHash() {
        this.spatialHash = new Map();
        const HC = this.HASH_CELL;
        for (let i = 0; i < this.modules.length; i++) {
            const m = this.modules[i];
            const key = (Math.floor(m.x / HC) * 100003 + Math.floor(m.y / HC)) | 0;
            let b = this.spatialHash.get(key);
            if (!b) { b = []; this.spatialHash.set(key, b); }
            b.push(i);
        }
    }

    _neighborsWithin(mi, r) {
        const HC = this.HASH_CELL;
        const m = this.modules[mi];
        const cx = Math.floor(m.x / HC);
        const cy = Math.floor(m.y / HC);
        const r2 = r * r;
        const out = [];
        for (let dx = -1; dx <= 1; dx++) {
            for (let dy = -1; dy <= 1; dy++) {
                const b = this.spatialHash.get(((cx + dx) * 100003 + (cy + dy)) | 0);
                if (!b) continue;
                for (const j of b) {
                    if (j === mi) continue;
                    const mj = this.modules[j];
                    if ((mj.x - m.x) ** 2 + (mj.y - m.y) ** 2 <= r2) out.push(j);
                }
            }
        }
        return out;
    }

    // ---- token diffusion ----
    // Phase-2 correction: distribute each module's tokens to ALL active neighbors
    // every round (Ishida's "tokens distributed equally to neighboring cells,
    // value +1 per hop", with a residual rate), instead of one random neighbor.
    // The single-neighbor scheme made the per-step potential extremely noisy,
    // which broke the potential-gradient cohesion; all-neighbor exchange yields a
    // smooth, center-high gradient that reproduces Ishida's behaviour.
    _diffuseTokens() {
        const N = this.modules.length;
        const TM = this.p.TOKEN_MAX;
        const resid = this.p.DIFFUSE_RESID;
        const noise = this.p.NOISE_ON && this.p.NOISE_SIGMA > 0;

        for (let i = 0; i < N; i++) {
            this.modules[i].tokens.fill(0);
            if (this.modules[i].active) this.modules[i].tokens[1] = 1;
        }
        if (this._diffNext.length !== N) {
            this._diffNext = Array.from({ length: N }, () => new Float64Array(TM + 2));
        }

        // positions are fixed during diffusion -> compute active neighbor lists once
        const neighLists = new Array(N);
        for (let i = 0; i < N; i++) {
            if (!this.modules[i].active) { neighLists[i] = null; continue; }
            neighLists[i] = this._neighborsWithin(i, this.p.R_NEIGH)
                                .filter(j => this.modules[j].active);
        }

        for (let s = 0; s < this.p.DIFFUSE_STEPS; s++) {
            for (let i = 0; i < N; i++) this._diffNext[i].fill(0);
            for (let i = 0; i < N; i++) {
                const m = this.modules[i];
                if (!m.active) continue;
                const neigh = neighLists[i];
                const deg = neigh.length;
                for (let n = 1; n <= TM - 1; n++) {
                    let t = m.tokens[n];
                    if (t === 0) continue;
                    if (noise) t = Math.max(0, t + this._gaussianNoise(this.p.NOISE_SIGMA));
                    this._diffNext[i][n] += t * resid;             // residual stays, same age
                    if (deg > 0) {
                        const share = t * (1 - resid) / deg;        // spread, aged +1
                        for (const j of neigh) this._diffNext[j][n + 1] += share;
                    } else {
                        this._diffNext[i][n] += t * (1 - resid);    // isolated: nothing to spread
                    }
                }
            }
            for (let i = 0; i < N; i++) this.modules[i].tokens.set(this._diffNext[i]);
        }
    }

    // Phase-2 correction: Ishida eq.(2) is Potential = N1 - w*N2 with
    // N1 = count of ALL tokens (values 1..2L) and N2 = count of INNER tokens
    // (values 1..L). The earlier port had these swapped (inner - w*all), which
    // inverted the gradient. Restored to the Ishida-faithful form here.
    _computePotentials() {
        const alpha = this.p.EMA_ALPHA;
        const TM = this.p.TOKEN_MAX, L = this.p.SUM2_LIMIT, w = this.p.W_MORPH;
        for (const m of this.modules) {
            let all = 0, inner = 0;
            for (let n = 1; n <= TM; n++) {
                const t = m.tokens[n];
                all += t;
                if (n <= L) inner += t;
            }
            const raw = all - inner * w;
            m.potential = (1 - alpha) * m.potential + alpha * raw;
        }
    }

    _updateDormancy() {
        for (let i = 0; i < this.modules.length; i++) {
            const m = this.modules[i];
            if (!m.active) {
                for (const j of this._neighborsWithin(i, this.p.R_NEIGH)) {
                    if (this.modules[j].active && this.modules[j].potential >= 0) {
                        m.active = true;
                        break;
                    }
                }
            }
        }
    }

    _updateLightSource() {
        const A = this.p.LIGHT_AMP;
        if (this.p.LIGHT_ON && this.p.LIGHT_MODE !== 'static') {
            if (this.p.LIGHT_MODE === 'oscillate') {
                this.lightSource.x = this.lightCenter.x + Math.sin(this.lightAngle) * A;
                this.lightSource.y = this.lightCenter.y;
            } else {
                this.lightSource.x = this.lightCenter.x + Math.cos(this.lightAngle) * A;
                this.lightSource.y = this.lightCenter.y + Math.sin(this.lightAngle) * A;
            }
            this.lightAngle += this.p.LIGHT_SPEED;
        }
        if (this.p.LIGHT2_ON && this.p.LIGHT2_MODE !== 'static') {
            if (this.p.LIGHT2_MODE === 'oscillate') {
                this.lightSource2.x = this.lightCenter2.x + Math.sin(this.lightAngle2) * A;
                this.lightSource2.y = this.lightCenter2.y;
            } else {
                this.lightSource2.x = this.lightCenter2.x + Math.cos(this.lightAngle2) * A;
                this.lightSource2.y = this.lightCenter2.y + Math.sin(this.lightAngle2) * A;
            }
            this.lightAngle2 += this.p.LIGHT2_SPEED;
        }
    }

    _computeShadowForLight(lx, ly, prop) {
        const N = this.modules.length;
        for (let i = 0; i < N; i++) this.modules[i][prop] = false;
        for (let i = 0; i < N; i++) {
            const mi = this.modules[i];
            if (!mi.active) continue;
            const rix = mi.x - lx, riy = mi.y - ly;
            const dist_i2 = rix * rix + riy * riy;
            if (dist_i2 < 1e-10) continue;
            for (let j = 0; j < N; j++) {
                if (j === i) continue;
                const mj = this.modules[j];
                if (!mj.active) continue;
                const rjx = mj.x - lx, rjy = mj.y - ly;
                const t = (rjx * rix + rjy * riy) / dist_i2;
                if (t <= 0 || t >= 1) continue;
                const px = rjx - t * rix, py = rjy - t * riy;
                if (px * px + py * py < mj.radius * mj.radius) { mi[prop] = true; break; }
            }
        }
    }

    _computeShadows() {
        if (this.p.LIGHT_ON) this._computeShadowForLight(this.lightSource.x, this.lightSource.y, 'inShadow');
        else for (const m of this.modules) m.inShadow = false;
        if (this.p.LIGHT2_ON) this._computeShadowForLight(this.lightSource2.x, this.lightSource2.y, 'inShadow2');
        else for (const m of this.modules) m.inShadow2 = false;
    }

    _applyForces() {
        const P = this.p;
        const MAX_R = P.HET_ON ? P.MAX_R_MOD : P.R_MOD;
        const noise = P.NOISE_ON && P.NOISE_SIGMA > 0;

        for (let i = 0; i < this.modules.length; i++) {
            const m = this.modules[i];

            if (!m.active) {
                const angle = this.random() * 2 * Math.PI;
                const dspd = P.MAX_STEP * 0.4;
                m.vx = m.vx * P.FRICTION + Math.cos(angle) * dspd * (1 - P.FRICTION);
                m.vy = m.vy * P.FRICTION + Math.sin(angle) * dspd * (1 - P.FRICTION);
                continue;
            }

            if (this.iterate - m.lastCollisionStep >= P.COLLISION_RESET_STEPS) m.collisions = 0;

            let fx = 0, fy = 0;

            // overlap repulsion
            const touchSearch = m.radius + MAX_R;
            for (const j of this._neighborsWithin(i, touchSearch)) {
                const n = this.modules[j];
                const TOUCH_IJ = m.radius + n.radius;
                const dx = n.x - m.x, dy = n.y - m.y;
                const d = Math.hypot(dx, dy) + 1e-6;
                if (d >= TOUCH_IJ) continue;
                m.collisions++;
                m.lastCollisionStep = this.iterate;
                const push = (d - TOUCH_IJ) * 8;
                fx += push * dx / d;
                fy += push * dy / d;
            }

            if (P.COLLISION_FAIL_ON && m.collisions >= P.COLLISION_THRESHOLD) {
                m.active = false; m.collisions = 0; m.vx = 0; m.vy = 0;
                this.totalCollisionDeaths++; this.stepDeaths++;
                continue;
            }

            // potential-gradient attraction
            const pSelf = Math.abs(m.potential) + 1;
            for (const j of this._neighborsWithin(i, P.R_NEIGH)) {
                const n = this.modules[j];
                if (!n.active) continue;
                const dx = n.x - m.x, dy = n.y - m.y;
                const d = Math.hypot(dx, dy) + 1e-6;
                const ratio = (n.potential - m.potential) / pSelf;
                const s = ratio * P.P_ATTRACT * 0.025;
                fx += s * dx / d;
                fy += s * dy / d;
            }

            // light 1
            if (P.LIGHT_ON) {
                const shadowFactor = m.inShadow ? 0.40 : 1.0;
                let ldx = this.lightSource.x - m.x;
                let ldy = this.lightSource.y - m.y;
                if (noise) {
                    const angle = Math.atan2(ldy, ldx) + this._gaussianNoise(P.NOISE_SIGMA * 0.3);
                    ldx = Math.cos(angle); ldy = Math.sin(angle);
                } else {
                    const ld = Math.hypot(ldx, ldy) + 1e-6;
                    ldx /= ld; ldy /= ld;
                }
                fx += ldx * P.P_LIGHT * 0.08 * shadowFactor;
                fy += ldy * P.P_LIGHT * 0.08 * shadowFactor;
            }

            // light 2
            if (P.LIGHT2_ON) {
                const shadowFactor2 = m.inShadow2 ? 0.40 : 1.0;
                let l2dx = this.lightSource2.x - m.x;
                let l2dy = this.lightSource2.y - m.y;
                if (noise) {
                    const angle = Math.atan2(l2dy, l2dx) + this._gaussianNoise(P.NOISE_SIGMA * 0.3);
                    l2dx = Math.cos(angle); l2dy = Math.sin(angle);
                } else {
                    const ld = Math.hypot(l2dx, l2dy) + 1e-6;
                    l2dx /= ld; l2dy /= ld;
                }
                fx += l2dx * P.P_LIGHT * 0.08 * shadowFactor2;
                fy += l2dy * P.P_LIGHT * 0.08 * shadowFactor2;
            }

            // random
            fx += (this.random() * 2 - 1) * P.P_RANDOM * 0.20;
            fy += (this.random() * 2 - 1) * P.P_RANDOM * 0.20;

            // inertia
            m.vx = m.vx * P.FRICTION + fx * P.DT;
            m.vy = m.vy * P.FRICTION + fy * P.DT;
            const spd = Math.hypot(m.vx, m.vy);
            if (spd > P.MAX_STEP) { m.vx *= P.MAX_STEP / spd; m.vy *= P.MAX_STEP / spd; }
        }

        // displacement + wall + world bounds
        const gapHalf = P.WALL_GAP / 2;
        for (const m of this.modules) {
            let nx = m.x + m.vx;
            let ny = m.y + m.vy;

            if (P.WALL_ON || P.WALL2_ON) {
                const crossing = (m.x - P.WALL_X) * (nx - P.WALL_X) < 0;
                if (crossing) {
                    const t = (P.WALL_X - m.x) / (nx - m.x + 1e-9);
                    const yAtWall = m.y + t * (ny - m.y);
                    let blocked;
                    if (P.WALL_ON) {
                        blocked = Math.abs(yAtWall - P.WALL_GAP_Y) > gapHalf;
                    } else {
                        const inGap1 = Math.abs(yAtWall - (P.WALL_GAP_Y - P.WALL_GAP_SEP / 2)) <= gapHalf;
                        const inGap2 = Math.abs(yAtWall - (P.WALL_GAP_Y + P.WALL_GAP_SEP / 2)) <= gapHalf;
                        blocked = !inGap1 && !inGap2;
                    }
                    if (blocked) {
                        nx = (m.x < P.WALL_X) ? P.WALL_X - P.R_MOD : P.WALL_X + P.R_MOD;
                        m.vx = -m.vx * 0.3;
                    }
                }
            }

            m.x = Math.max(P.R_MOD, Math.min(P.WORLD_W - P.R_MOD, nx));
            m.y = Math.max(P.R_MOD, Math.min(P.WORLD_H - P.R_MOD, ny));
        }
    }

    computeMEI() {
        const act = this.modules.filter(m => m.active);
        if (act.length < 2) return 1;
        let cx = 0, cy = 0;
        for (const m of act) { cx += m.x; cy += m.y; }
        cx /= act.length; cy /= act.length;
        let s1 = 0;
        for (const m of act) s1 += Math.hypot(m.x - cx, m.y - cy);
        const avgR = act.reduce((s, m) => s + m.radius, 0) / act.length;
        // packing radius of an ideal hex-packed disc (fraction phi=0.907):
        // pi*Rpack^2 = N*pi*r^2/phi  ->  Rpack = r*sqrt(N/phi).
        // (The earlier browser code divided by an extra pi, so a perfect circle
        //  gave MEI~1.77 instead of 1.0; corrected here for Ishida-comparable scale.)
        const packR = avgR * Math.sqrt(act.length / 0.907);
        const s2 = act.length * (2 * packR / 3);
        return s1 / s2;
    }

    // center of gravity of active modules
    centerOfGravity() {
        const act = this.modules.filter(m => m.active);
        if (!act.length) return { x: NaN, y: NaN, n: 0 };
        let cx = 0, cy = 0;
        for (const m of act) { cx += m.x; cy += m.y; }
        return { x: cx / act.length, y: cy / act.length, n: act.length };
    }

    countActive() { return this.modules.reduce((s, m) => s + (m.active ? 1 : 0), 0); }

    step() {
        this.iterate++;
        this.stepDeaths = 0;
        this._rebuildHash();
        this._diffuseTokens();
        this._computePotentials();
        this._updateDormancy();
        this._updateLightSource();
        this._computeShadows();
        this._applyForces();
        const mei = this.computeMEI();
        if (this.recoveryActive && this.recoveryTime === -1 && mei <= this.recoveryMEI) {
            this.recoveryTime = this.iterate - this.recoveryStep;
            this.recoveryActive = false;
        }
        return mei;
    }
}

module.exports = { Sim, mulberry32, DEFAULTS };
