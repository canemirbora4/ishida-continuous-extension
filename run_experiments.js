//
// run_experiments.js
// Batch runner for the 8 planned experiments of the Phase-2 paper.
// Uses sim_core.js (headless, seeded). Writes CSV files + summary.json to results/.
//
// Usage:  node run_experiments.js [exp1 exp2 ...]   (default: all)
//

'use strict';
const fs = require('fs');
const path = require('path');
const { Sim } = require('./sim_core.js');

const OUT = path.join(__dirname, 'results');
if (!fs.existsSync(OUT)) fs.mkdirSync(OUT);

function writeCSV(name, header, rows) {
    const lines = [header.join(',')];
    for (const r of rows) lines.push(r.join(','));
    fs.writeFileSync(path.join(OUT, name), lines.join('\n') + '\n');
    console.log('  wrote', name, '(' + rows.length + ' rows)');
}

function mean(a) { return a.length ? a.reduce((s, x) => s + x, 0) / a.length : NaN; }
function std(a) {
    if (a.length < 2) return 0;
    const m = mean(a);
    return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / (a.length - 1));
}
function round(x, d = 4) { return Number.isFinite(x) ? +x.toFixed(d) : x; }

const summary = {};

// distance from swarm center of gravity to light source
function distToLight(s) {
    const c = s.centerOfGravity();
    return Math.hypot(c.x - s.lightSource.x, c.y - s.lightSource.y);
}

// ---------------------------------------------------------------------------
// EXPERIMENT 1 — Baseline navigation (static light)
//   5 seeds, 3000 steps, MEI every 100 steps, navigation time (COG within 5).
// ---------------------------------------------------------------------------
function exp1() {
    console.log('Experiment 1: baseline navigation');
    const SEEDS = [1, 2, 3, 4, 5];
    const STEPS = 3000, SAMPLE = 100, NAV_THRESH = 5;
    const meiSeries = {};          // step -> [mei per seed]
    const navTimes = [];

    for (const seed of SEEDS) {
        const s = new Sim({ seed });
        s.initCluster();
        let nav = -1;
        for (let t = 1; t <= STEPS; t++) {
            const mei = s.step();
            if (nav < 0 && distToLight(s) <= NAV_THRESH) nav = t;
            if (t % SAMPLE === 0) {
                (meiSeries[t] = meiSeries[t] || []).push(mei);
            }
        }
        navTimes.push(nav < 0 ? STEPS : nav);
    }

    const rows = [];
    for (let t = SAMPLE; t <= STEPS; t += SAMPLE) {
        rows.push([t, round(mean(meiSeries[t])), round(std(meiSeries[t]))]);
    }
    writeCSV('exp1_mei_timeseries.csv', ['step', 'mei_mean', 'mei_std'], rows);

    summary.exp1 = {
        nav_time_mean: round(mean(navTimes), 1),
        nav_time_std: round(std(navTimes), 1),
        nav_times: navTimes,
        mei_final_mean: round(mean(meiSeries[STEPS]), 3),
        mei_settled_mean: round(mean([1500, 2000, 2500, 3000].flatMap(t => meiSeries[t])), 3),
    };
    console.log('  nav time:', summary.exp1.nav_time_mean, '+/-', summary.exp1.nav_time_std,
                '| settled MEI:', summary.exp1.mei_settled_mean);
}

// ---------------------------------------------------------------------------
// EXPERIMENT 2 — Effect of friction gamma
//   gamma 0.50..0.95 step .05 plus 0.99, 3 trials, 2000 steps.
//   mean MEI over steps 500-2000, navigation time.
// ---------------------------------------------------------------------------
function exp2() {
    console.log('Experiment 2: friction sweep');
    const GAMMAS = [0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.99];
    const TRIALS = 3, STEPS = 2000, NAV_THRESH = 5;
    const rows = [];
    for (const g of GAMMAS) {
        const meis = [], navs = [];
        for (let tr = 0; tr < TRIALS; tr++) {
            const s = new Sim({ seed: 100 + tr, FRICTION: g });
            s.initCluster();
            const window = [];
            let nav = -1;
            for (let t = 1; t <= STEPS; t++) {
                const mei = s.step();
                if (nav < 0 && distToLight(s) <= NAV_THRESH) nav = t;
                if (t >= 500) window.push(mei);
            }
            meis.push(mean(window));
            navs.push(nav < 0 ? STEPS : nav);
        }
        rows.push([g, round(mean(meis), 3), round(std(meis), 3), round(mean(navs), 1), round(std(navs), 1)]);
        console.log('  gamma=' + g.toFixed(2), 'MEI=' + round(mean(meis), 3), 'nav=' + round(mean(navs), 0));
    }
    writeCSV('exp2_friction.csv', ['gamma', 'mei_mean', 'mei_std', 'nav_mean', 'nav_std'], rows);
    summary.exp2 = { gammas: GAMMAS, rows };
}

// ---------------------------------------------------------------------------
// EXPERIMENT 3 — Noise robustness
//   sigma 0..3 step 0.5, 3 trials, 1500 steps, mean MEI over steps 500-1000.
// ---------------------------------------------------------------------------
function exp3() {
    console.log('Experiment 3: noise robustness');
    const SIGMAS = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
    const TRIALS = 3, STEPS = 1500, NAV_THRESH = 5;
    const rows = [];
    for (const sigma of SIGMAS) {
        const meis = [], navs = [];
        for (let tr = 0; tr < TRIALS; tr++) {
            const s = new Sim({ seed: 200 + tr, NOISE_ON: sigma > 0, NOISE_SIGMA: sigma });
            s.initCluster();
            const window = [];
            let nav = -1;
            for (let t = 1; t <= STEPS; t++) {
                const mei = s.step();
                if (nav < 0 && distToLight(s) <= NAV_THRESH) nav = t;
                if (t >= 500 && t <= 1000) window.push(mei);
            }
            meis.push(mean(window));
            navs.push(nav < 0 ? STEPS : nav);
        }
        rows.push([sigma, round(mean(meis), 3), round(std(meis), 3), round(mean(navs), 1), round(std(navs), 1)]);
        console.log('  sigma=' + sigma.toFixed(1), 'MEI=' + round(mean(meis), 3), 'nav=' + round(mean(navs), 0));
    }
    writeCSV('exp3_noise.csv', ['sigma', 'mei_mean', 'mei_std', 'nav_mean', 'nav_std'], rows);
    summary.exp3 = { sigmas: SIGMAS, rows };
}

// ---------------------------------------------------------------------------
// EXPERIMENT 4 — Dynamic light tracking (oscillate mode)
//   omega 0.005..0.10, 3 trials, 2000 steps.
//   tracking error = mean |COG - light|, steady state (steps 500-2000).
// ---------------------------------------------------------------------------
function exp4() {
    console.log('Experiment 4: dynamic light tracking');
    const OMEGAS = [0.005, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10];
    const TRIALS = 3, STEPS = 2000;
    const rows = [];
    for (const w of OMEGAS) {
        const errFull = [], errSteady = [];
        for (let tr = 0; tr < TRIALS; tr++) {
            const s = new Sim({ seed: 300 + tr, LIGHT_MODE: 'oscillate', LIGHT_SPEED: w });
            s.initCluster();
            const ef = [], es = [];
            for (let t = 1; t <= STEPS; t++) {
                s.step();
                const d = distToLight(s);
                ef.push(d);
                if (t >= 500) es.push(d);
            }
            errFull.push(mean(ef));
            errSteady.push(mean(es));
        }
        rows.push([w, round(mean(errSteady), 3), round(std(errSteady), 3), round(mean(errFull), 3)]);
        console.log('  omega=' + w.toFixed(3), 'track_err(steady)=' + round(mean(errSteady), 2));
    }
    writeCSV('exp4_tracking.csv', ['omega', 'err_steady_mean', 'err_steady_std', 'err_full_mean'], rows);
    summary.exp4 = { omegas: OMEGAS, rows };
}

// ---------------------------------------------------------------------------
// EXPERIMENT 5 — Failure and recovery
//   (a) converge to step 1500, inject Kill-20% at 1500/2000/2500, record
//       recovery time after each event.
//   (b) collision-fatigue test: theta=20, dense cluster, count deaths.
// ---------------------------------------------------------------------------
function exp5() {
    console.log('Experiment 5: failure and recovery');
    const SEEDS = [1, 2, 3, 4, 5];
    const KILL_STEPS = [1500, 2000, 2500];
    const END = 3000;
    const recRows = [];          // seed, event#, killStep, meiBefore, recoveryTime, recovered
    const recByEvent = [[], [], []];

    for (const seed of SEEDS) {
        const s = new Sim({ seed });
        s.initCluster();
        let evtIdx = 0;
        // single pending recovery tracker (kills are 500 apart; if not recovered
        // before the next kill, the pending event is closed as "not recovered").
        let pendTarget = 0, pendStart = 0, pendEvt = 0, pending = false;

        const closePending = (recovered, recTime) => {
            recRows.push([seed, pendEvt, pendStart, round(pendTarget, 3),
                          recTime, recovered ? 'yes' : 'no']);
            recByEvent[pendEvt - 1].push(recTime);
            pending = false;
        };

        for (let t = 1; t <= END; t++) {
            // trigger a kill at scheduled steps
            if (evtIdx < KILL_STEPS.length && t === KILL_STEPS[evtIdx]) {
                if (pending) closePending(false, t - pendStart);   // overlap -> not recovered
                pendTarget = s.computeMEI();
                s.killRandom20();
                pendStart = t;
                pendEvt = evtIdx + 1;
                pending = true;
                evtIdx++;
            }
            const mei = s.step();
            if (pending && mei <= pendTarget) closePending(true, t - pendStart);
        }
        if (pending) closePending(false, END - pendStart);   // never recovered by END
    }
    writeCSV('exp5a_recovery.csv',
        ['seed', 'event', 'kill_step', 'mei_before', 'recovery_steps', 'recovered'], recRows);

    // (b) collision fatigue
    const fatRows = [];
    for (const seed of SEEDS) {
        const s = new Sim({ seed, COLLISION_FAIL_ON: true, COLLISION_THRESHOLD: 20,
                            N_INIT: 120, INIT_RADIUS: 4 });   // dense cluster
        s.initCluster();
        for (let t = 1; t <= 2000; t++) {
            s.step();
            if (t % 200 === 0) fatRows.push([seed, t, s.countActive(), s.totalCollisionDeaths, round(s.computeMEI(), 3)]);
        }
    }
    writeCSV('exp5b_fatigue.csv', ['seed', 'step', 'active', 'deaths', 'mei'], fatRows);

    summary.exp5 = {
        recovery_event_means: recByEvent.map(a => round(mean(a), 1)),
        recovery_event_stds: recByEvent.map(a => round(std(a), 1)),
        recovery_all_mean: round(mean(recByEvent.flat()), 1),
    };
    console.log('  recovery per event (mean steps):', summary.exp5.recovery_event_means);
}

// ---------------------------------------------------------------------------
// EXPERIMENT 6 — Heterogeneous modules
//   het on, 5 seeds, 3000 steps. MEI trajectory + nav time vs homogeneous.
//   size sorting: mean normalized dist-to-COG for S / M / L modules.
// ---------------------------------------------------------------------------
function exp6() {
    console.log('Experiment 6: heterogeneous modules');
    const SEEDS = [1, 2, 3, 4, 5];
    const STEPS = 3000, SAMPLE = 100, NAV_THRESH = 5;
    const meiSeries = {};
    const navTimes = [];
    const sortS = [], sortM = [], sortL = [];   // mean dist-to-COG by tier, last 500 steps

    for (const seed of SEEDS) {
        const s = new Sim({ seed, HET_ON: true });
        s.initCluster();
        let nav = -1;
        const accS = [], accM = [], accL = [];
        for (let t = 1; t <= STEPS; t++) {
            const mei = s.step();
            if (nav < 0 && distToLight(s) <= NAV_THRESH) nav = t;
            if (t % SAMPLE === 0) (meiSeries[t] = meiSeries[t] || []).push(mei);
            if (t > STEPS - 500) {
                const c = s.centerOfGravity();
                for (const m of s.modules) {
                    if (!m.active) continue;
                    const d = Math.hypot(m.x - c.x, m.y - c.y);
                    if (m.radius <= 0.35) accS.push(d);
                    else if (m.radius <= 0.60) accM.push(d);
                    else accL.push(d);
                }
            }
        }
        navTimes.push(nav < 0 ? STEPS : nav);
        sortS.push(mean(accS)); sortM.push(mean(accM)); sortL.push(mean(accL));
    }

    const rows = [];
    for (let t = SAMPLE; t <= STEPS; t += SAMPLE) {
        rows.push([t, round(mean(meiSeries[t])), round(std(meiSeries[t]))]);
    }
    writeCSV('exp6_het_mei_timeseries.csv', ['step', 'mei_mean', 'mei_std'], rows);

    summary.exp6 = {
        nav_time_mean: round(mean(navTimes), 1),
        nav_time_std: round(std(navTimes), 1),
        mei_settled_mean: round(mean([2000, 2500, 3000].flatMap(t => meiSeries[t])), 3),
        sort_dist_S: round(mean(sortS), 2),
        sort_dist_M: round(mean(sortM), 2),
        sort_dist_L: round(mean(sortL), 2),
    };
    console.log('  nav:', summary.exp6.nav_time_mean, '| settled MEI:', summary.exp6.mei_settled_mean,
                '| sort S/M/L:', summary.exp6.sort_dist_S, summary.exp6.sort_dist_M, summary.exp6.sort_dist_L);
}

// ---------------------------------------------------------------------------
// EXPERIMENT 7 — Single-slit gap traversal
//   wall on, static light right. gap 7..4 step 1, 3 trials, 3000 steps.
//   record: success, max MEI during traversal, final MEI, fraction crossed.
// ---------------------------------------------------------------------------
function exp7() {
    console.log('Experiment 7: single-slit gap traversal');
    const GAPS = [7, 6, 5, 4];
    const TRIALS = 3, STEPS = 3000, NAV_THRESH = 6;
    const rows = [];
    for (const gap of GAPS) {
        const succ = [], maxMEI = [], finMEI = [], crossFrac = [], navT = [];
        for (let tr = 0; tr < TRIALS; tr++) {
            const s = new Sim({ seed: 700 + tr, WALL_ON: true, WALL_GAP: gap });
            s.initCluster();
            let mmax = 0, nav = -1;
            for (let t = 1; t <= STEPS; t++) {
                const mei = s.step();
                if (mei > mmax) mmax = mei;
                if (nav < 0 && distToLight(s) <= NAV_THRESH) nav = t;
            }
            const act = s.modules.filter(m => m.active);
            const crossed = act.filter(m => m.x > s.p.WALL_X).length / Math.max(1, act.length);
            const reached = distToLight(s) <= NAV_THRESH ? 1 : 0;
            succ.push(reached);
            maxMEI.push(mmax);
            finMEI.push(s.computeMEI());
            crossFrac.push(crossed);
            navT.push(nav < 0 ? STEPS : nav);
        }
        rows.push([gap, round(mean(succ), 2), round(mean(maxMEI), 3), round(mean(finMEI), 3),
                   round(mean(crossFrac), 3), round(mean(navT), 0)]);
        console.log('  gap=' + gap, 'success=' + round(mean(succ), 2), 'maxMEI=' + round(mean(maxMEI), 2),
                    'crossed=' + round(mean(crossFrac), 2));
    }
    writeCSV('exp7_singleslit.csv',
        ['gap', 'success_rate', 'max_mei', 'final_mei', 'crossed_frac', 'nav_steps'], rows);
    summary.exp7 = { gaps: GAPS, rows };
}

// ---------------------------------------------------------------------------
// EXPERIMENT 8 — Double-slit behavior
//   wall2 on. (a) two lights symmetric about y=30 on the right.
//             (b) single centered light.
//   5 seeds, 2500 steps. fraction of active modules above/below y=30 after
//   passing wall (x>70), final MEI, split index.
// ---------------------------------------------------------------------------
function exp8() {
    console.log('Experiment 8: double-slit behavior');
    const SEEDS = [1, 2, 3, 4, 5];
    const STEPS = 2500;
    const sep = 16;                       // slit separation (matches WALL_GAP_SEP)
    const yTop = 30 - sep / 2, yBot = 30 + sep / 2;

    function runCase(twoLights) {
        const rows = [];
        const aggSplit = [], aggMEI = [], aggCrossed = [];
        for (const seed of SEEDS) {
            const cfg = { seed, WALL2_ON: true };
            if (twoLights) {
                cfg.LIGHT_X = 105; cfg.LIGHT_Y = yTop;
                cfg.LIGHT2_ON = true; cfg.LIGHT2_X = 105; cfg.LIGHT2_Y = yBot;
            } else {
                cfg.LIGHT_X = 105; cfg.LIGHT_Y = 30;
            }
            const s = new Sim(cfg);
            s.initCluster();
            for (let t = 1; t <= STEPS; t++) s.step();
            const past = s.modules.filter(m => m.active && m.x > s.p.WALL_X);
            const above = past.filter(m => m.y < 30).length;
            const below = past.length - above;
            const crossed = past.length / Math.max(1, s.countActive());
            // split index: 0 = all one side, 1 = perfectly balanced
            const split = past.length ? (1 - Math.abs(above - below) / past.length) : 0;
            const mei = s.computeMEI();
            rows.push([seed, twoLights ? 2 : 1, above, below, round(crossed, 3), round(split, 3), round(mei, 3)]);
            aggSplit.push(split); aggMEI.push(mei); aggCrossed.push(crossed);
        }
        return { rows, split: round(mean(aggSplit), 3), mei: round(mean(aggMEI), 3),
                 crossed: round(mean(aggCrossed), 3) };
    }

    const two = runCase(true);
    const one = runCase(false);
    writeCSV('exp8_doubleslit.csv',
        ['seed', 'lights', 'above', 'below', 'crossed_frac', 'split_index', 'final_mei'],
        [...two.rows, ...one.rows]);
    summary.exp8 = {
        two_lights: { split_index: two.split, final_mei: two.mei, crossed_frac: two.crossed },
        one_light:  { split_index: one.split, final_mei: one.mei, crossed_frac: one.crossed },
    };
    console.log('  two-light split=' + two.split, 'crossed=' + two.crossed,
                '| one-light split=' + one.split, 'crossed=' + one.crossed);
}

// ---------------------------------------------------------------------------
const ALL = { exp1, exp2, exp3, exp4, exp5, exp6, exp7, exp8 };
const args = process.argv.slice(2);
const toRun = args.length ? args : Object.keys(ALL);
const t0 = Date.now();
for (const k of toRun) {
    if (!ALL[k]) { console.error('unknown experiment', k); continue; }
    ALL[k]();
}
fs.writeFileSync(path.join(OUT, 'summary.json'), JSON.stringify(summary, null, 2));
console.log('\nDone in', ((Date.now() - t0) / 1000).toFixed(1), 's. Summary -> results/summary.json');
