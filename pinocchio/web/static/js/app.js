import { RobotViewer, AXIS_COLORS } from './viewer.js';

const NJ = 6;
const CART_AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const JOG_REFRESH_MS = 60;          // must stay under the controller's 200 ms dead-man
const TABS = [['jog', 'Jog'], ['joints', 'Joints'], ['poses', 'Poses'], ['setup', 'Setup']];

const $ = (id) => document.getElementById(id);
const rad2deg = 180 / Math.PI;
const axColor = (a) => '#' + AXIS_COLORS[a % 3].toString(16).padStart(6, '0');

const DEFAULTS = {
    jogMode: 'hold', cartFrame: 'base',
    jogSpeed: 20, cartLinSpeed: 0.05, cartAngSpeed: 15,
    selInc: 10, cartLinStep: 0.01, cartAngStep: 5,
    trail: true, tab: 'jog',
};
let S = { ...DEFAULTS };

let state = {
    pos: Array(NJ).fill(0), tgt: Array(NJ).fill(0), tcp: Array(6).fill(0),
    vmax: Array(NJ).fill(0), amax: Array(NJ).fill(0), jmax: Array(NJ).fill(0),
    enabled: Array(NJ).fill(1), sigma: 1, frame: 'base',
};
let config = { poses: {}, sequences: {}, increments: [5, 10, 20], settings: {} };
let jointLimits = null;
let curSteps = [];
const sliderBusy = Array(NJ).fill(0);
const jogTimers = Array(NJ).fill(null);
const cartJogTimers = Array(6).fill(null);

// ------------------------------------------------------------------ transport
let ws = null;
function connect() {
    ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
    ws.onopen = () => setStatus('connected', 'ok');
    ws.onclose = () => { setStatus('disconnected — retrying', 'err'); setTimeout(connect, 1000); };
    ws.onmessage = (e) => {
        const m = JSON.parse(e.data);
        if (m.type === 'state') onState(m);
        else if (m.type === 'config') onConfig(m);
        else if (m.type === 'play_done') toast('Sequence finished', 'good');
    };
}
function send(o) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(o));
    else setStatus('controller unavailable', 'err');
}
const cmd = (line) => send({ type: 'cmd', line });
function setStatus(t, c) { const s = $('status'); s.textContent = t; s.className = c || ''; }

function toast(msg, kind = '', ms = 3000) {
    const el = document.createElement('div');
    el.className = 'toast ' + kind;
    el.textContent = msg;
    $('toasts').appendChild(el);
    setTimeout(() => el.remove(), ms);
}

// ------------------------------------------------------------------ settings
function onConfig(m) {
    config = { increments: [5, 10, 20], settings: {}, ...m };
    S = { ...DEFAULTS, ...(config.settings || {}) };
    applySettings();
    renderAll();
}
function applySettings() {
    $('jogSpeed').value = S.jogSpeed;
    $('cartLinSpeed').value = S.cartLinSpeed;
    $('cartAngSpeed').value = S.cartAngSpeed;
    viewer.setFrame(S.cartFrame);
    viewer.setTrail(S.trail);
    $('btnTrail').classList.toggle('on', S.trail);
    // Config is re-broadcast on every pose/sequence edit, so only nudge the
    // controller when the frame actually differs from what it reports.
    if (state.frame !== S.cartFrame) cmd(`cartframe ${S.cartFrame}`);
    showTab(S.tab);
}
function saveSettings() {
    send({ type: 'save_settings', values: { ...S } });
    toast('Settings saved', 'good', 1800);
}
function resetSettings() {
    S = { ...DEFAULTS };
    applySettings();
    renderAll();
    send({ type: 'save_settings', values: { ...S } });
    toast('Settings reset', 'good', 1800);
}

// -------------------------------------------------------------------- viewer
const viewer = new RobotViewer($('viewport'), {
    onDragJog: (axis, rate) => dragJog(axis, rate),
    onDragEnd: () => { stopAllCartJogs(); hideHud(); },
    onPlanChange: (p) => renderPlan(p),
});

window.parol = { viewer };   // debug handle: inspect frames from the browser console

viewer.load('/assets/parol6.urdf').then(() => {
    $('loadMsg').remove();
    jointLimits = viewer.jointLimits();
    buildJoints();
    viewer.setFrame(S.cartFrame);
}).catch(err => {
    $('loadMsg').textContent = 'failed to load model: ' + err;
    console.error(err);
});

// A drag on a TCP handle produces a -1..1 rate; scale it by the same speed the
// on-screen jog buttons use so both paths feel identical.
function dragJog(axis, rate) {
    const max = axis < 3 ? S.cartLinSpeed : S.cartAngSpeed / rad2deg;
    startCartJogVel(axis, rate * max);
    showHud(axis, rate);
}

// --------------------------------------------------------------- speed HUD
// Shows how fast the arm is being asked to move while an axis is dragged or
// held, so a small drag visibly reads as a small commanded speed.
function showHud(axis, rate) {
    const hud = $('jogHud');
    hud.hidden = false;
    hud.style.setProperty('--ax', axColor(axis));
    $('hudAx').textContent = CART_AXES[axis].toUpperCase();
    const mag = Math.min(1, Math.abs(rate));
    const fill = $('hudFill');
    fill.style.width = (mag * 50) + '%';
    fill.style.left = rate >= 0 ? '50%' : (50 - mag * 50) + '%';
    const v = axis < 3
        ? `${(rate * S.cartLinSpeed).toFixed(4)} m/s`
        : `${(rate * S.cartAngSpeed).toFixed(1)} °/s`;
    $('hudVal').textContent = v;
}
function hideHud() { $('jogHud').hidden = true; }

// ------------------------------------------------------------------- state in
let lastSigmaWarn = 0;
function onState(m) {
    state = { ...state, ...m, enabled: Array.isArray(m.enabled) ? m.enabled : state.enabled };
    viewer.setJoints(m.pos);

    for (let j = 0; j < NJ; j++) {
        const ac = $(`ac${j}`); if (ac) ac.textContent = (+m.pos[j]).toFixed(1);
        if (Date.now() - sliderBusy[j] > 700) {
            const sl = $(`sl${j}`), tg = $(`tg${j}`);
            if (sl) sl.value = m.tgt[j];
            if (tg && document.activeElement !== tg) tg.value = (+m.tgt[j]).toFixed(1);
        }
        setIfIdle(`v${j}`, m.vmax[j]); setIfIdle(`a${j}`, m.amax[j]); setIfIdle(`k${j}`, m.jmax[j]);
    }

    if (Array.isArray(m.tcp)) renderReadout(m.tcp, m.sigma);
    if (m.frame && m.frame !== S.cartFrame) { S.cartFrame = m.frame; viewer.setFrame(m.frame); renderFrameBars(); }

    if (typeof m.sigma === 'number' && m.sigma < 0.02 && Date.now() - lastSigmaWarn > 5000) {
        lastSigmaWarn = Date.now();
        toast(`Near singularity (σmin ${m.sigma.toFixed(4)}) — Cartesian motion may be blocked`, 'warn', 4000);
    }
    renderMotors();
}
function setIfIdle(id, v) { const el = $(id); if (el && document.activeElement !== el) el.value = v; }

function renderReadout(t, sigma) {
    const cls = sigma < 0.02 ? 'bad' : sigma < 0.05 ? 'warn' : 'good';
    $('vpReadout').innerHTML =
        `<b>X</b><span class="v">${t[0].toFixed(4)} m</span>` +
        `<b>Y</b><span class="v">${t[1].toFixed(4)} m</span>` +
        `<b>Z</b><span class="v">${t[2].toFixed(4)} m</span>` +
        `<b>RX</b><span class="v">${(t[3] * rad2deg).toFixed(1)}°</span>` +
        `<b>RY</b><span class="v">${(t[4] * rad2deg).toFixed(1)}°</span>` +
        `<b>RZ</b><span class="v">${(t[5] * rad2deg).toFixed(1)}°</span>` +
        `<b>σmin</b><span class="v badge ${cls}">${(sigma ?? 0).toFixed(4)}</span>`;
    for (let a = 0; a < 6; a++) {
        const el = $(`cval${a}`);
        if (el) el.textContent = a < 3 ? `${t[a].toFixed(4)} m` : `${(t[a] * rad2deg).toFixed(1)}°`;
    }
}

// ------------------------------------------------------------ cartesian jog
function arrowSVG(dir, rot) {
    if (!rot) {
        const p = dir > 0 ? 'M5 12h13M13 7l5 5-5 5' : 'M19 12H6M11 7l-5 5 5 5';
        return `<svg width="24" height="24" viewBox="0 0 24 24"><path d="${p}"/></svg>`;
    }
    const arc = 'M5 13a7 7 0 0 1 14 0';
    const head = dir > 0 ? 'M19 13l-3-3M19 13l3-3' : 'M5 13l-3-3M5 13l3-3';
    const g = dir > 0 ? '' : ' transform="scale(-1,1) translate(-24,0)"';
    return `<svg width="24" height="24" viewBox="0 0 24 24"><g${g}><path d="${arc}"/><path d="${head}"/></g></svg>`;
}

// Wires a press-and-hold jog button. The release listener lives on the window so
// letting go outside the button still stops motion; pointer capture is avoided
// because a capture failure would otherwise abort the handler mid-jog.
function bindHold(btn, { onStart, onStop, onTap, onEnter, onLeave }) {
    if (onEnter) btn.addEventListener('pointerenter', onEnter);
    if (onLeave) btn.addEventListener('pointerleave', onLeave);
    btn.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        btn.classList.add('active');
        const held = S.jogMode === 'hold';
        if (held) onStart();
        const release = (ev) => {
            window.removeEventListener('pointerup', release);
            window.removeEventListener('pointercancel', release);
            btn.classList.remove('active');
            if (held) onStop();
            else if (ev.type === 'pointerup' && btn.contains(ev.target)) onTap();
        };
        window.addEventListener('pointerup', release);
        window.addEventListener('pointercancel', release);
    });
}

function buildCartPad(rootId, axes) {
    const root = $(rootId);
    root.innerHTML = '';
    axes.forEach(a => {
        const rot = a >= 3;
        const row = document.createElement('div');
        row.className = 'jogrow';
        row.style.setProperty('--ax', axColor(a));
        row.innerHTML =
            `<span class="lbl">${CART_AXES[a].toUpperCase()}</span>` +
            `<div class="jogbtn" data-cart="${a}:-1">${arrowSVG(-1, rot)}</div>` +
            `<span class="val" id="cval${a}">—</span>` +
            `<div class="jogbtn" data-cart="${a}:1">${arrowSVG(1, rot)}</div>`;
        root.appendChild(row);

        row.querySelectorAll('[data-cart]').forEach(btn => {
            const [ax, dir] = btn.dataset.cart.split(':').map(Number);
            bindHold(btn, {
                onStart: () => { startCartJog(ax, dir); showHud(ax, dir); },
                onStop: () => { stopCartJog(ax); hideHud(); },
                onTap: () => stepCartJog(ax, dir),
                onEnter: () => viewer.highlightAxis(ax),
                onLeave: () => viewer.highlightAxis(null),
            });
        });
    });
}

function stepCartJog(a, dir) {
    const d = a < 3 ? S.cartLinStep : S.cartAngStep / rad2deg;
    cmd(`cartjog ${CART_AXES[a]} ${(dir * d).toFixed(5)}`);
}
function sendCartJogVel(a, v) { cmd(`cartjogvel ${CART_AXES[a]} ${v.toFixed(4)}`); }
function startCartJog(a, dir) {
    startCartJogVel(a, dir * (a < 3 ? S.cartLinSpeed : S.cartAngSpeed / rad2deg));
}
function startCartJogVel(a, v) {
    sendCartJogVel(a, v);
    if (cartJogTimers[a]) clearInterval(cartJogTimers[a]);
    cartJogTimers[a] = setInterval(() => sendCartJogVel(a, v), JOG_REFRESH_MS);
}
function stopCartJog(a) {
    if (!cartJogTimers[a]) return;
    clearInterval(cartJogTimers[a]); cartJogTimers[a] = null;
    sendCartJogVel(a, 0);
}
function stopAllCartJogs() { for (let a = 0; a < 6; a++) stopCartJog(a); }

// ------------------------------------------------------------------- joints
function buildJoints() {
    const root = $('joints');
    root.innerHTML = '';
    for (let j = 0; j < NJ; j++) {
        const lim = jointLimits ? jointLimits[j] : { lower: -180, upper: 180 };
        const r = document.createElement('div');
        r.className = 'jrow';
        r.innerHTML = `
            <span class="jname">J${j + 1}</span>
            <button class="btn sm" data-jog="${j}:-1">−</button>
            <input type="range" id="sl${j}" min="${lim.lower.toFixed(1)}" max="${lim.upper.toFixed(1)}" step="0.1" value="0">
            <button class="btn sm" data-jog="${j}:1">＋</button>
            <input type="number" id="tg${j}" class="num" step="0.1">
            <span class="act" id="ac${j}">0.0</span>
            <button class="btn sm" data-zero="${j}" title="joint to 0">0</button>`;
        root.appendChild(r);

        const sl = r.querySelector(`#sl${j}`);
        sl.addEventListener('input', () => { sliderBusy[j] = Date.now(); setJoint(j, sl.value); });
        r.querySelector(`#tg${j}`).addEventListener('change', (e) => setJoint(j, e.target.value));
        r.querySelector('[data-zero]').addEventListener('click', () => cmd(`${j + 1} 0`));
        r.querySelectorAll('[data-jog]').forEach(btn => {
            const [jj, dir] = btn.dataset.jog.split(':').map(Number);
            bindHold(btn, {
                onStart: () => startJog(jj, dir),
                onStop: () => stopJog(jj),
                onTap: () => stepJog(jj, dir),
            });
        });
    }
}
const setJoint = (j, v) => cmd(`${j + 1} ${(+v).toFixed(3)}`);
function stepJog(j, dir) { cmd(`jog ${j + 1} ${dir * S.selInc}`); sliderBusy[j] = Date.now(); }
function sendJogVel(j, v) { cmd(`jogvel ${j + 1} ${v.toFixed(2)}`); }
function startJog(j, dir) {
    sliderBusy[j] = Date.now();
    sendJogVel(j, dir * S.jogSpeed);
    if (jogTimers[j]) clearInterval(jogTimers[j]);
    jogTimers[j] = setInterval(() => sendJogVel(j, dir * S.jogSpeed), JOG_REFRESH_MS);
}
function stopJog(j) {
    if (!jogTimers[j]) return;
    clearInterval(jogTimers[j]); jogTimers[j] = null;
    sendJogVel(j, 0);
}
function stopAllJogs() { for (let j = 0; j < NJ; j++) stopJog(j); }

function stopEverything() { stopAllJogs(); stopAllCartJogs(); hideHud(); }
window.addEventListener('blur', stopEverything);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopEverything(); });

// ------------------------------------------------------------------- motors
function renderMotors() {
    const bar = $('motorBar'); bar.innerHTML = '';
    for (let j = 0; j < NJ; j++) {
        const on = !!state.enabled[j];
        const b = document.createElement('button');
        b.className = 'btn' + (on ? ' accent' : ' off');
        b.textContent = `J${j + 1}`;
        b.title = on ? 'torque on' : 'torque off';
        b.onclick = () => cmd(`motor ${j + 1} ${on ? 'off' : 'on'}`);
        bar.appendChild(b);
    }
}

// -------------------------------------------------------------------- limits
function buildLimits() {
    const b = $('limitBody'); b.innerHTML = '';
    for (let j = 0; j < NJ; j++) {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>J${j + 1}</td>
            <td><input id="v${j}" type="number" class="num" step="1"></td>
            <td><input id="a${j}" type="number" class="num" step="10"></td>
            <td><input id="k${j}" type="number" class="num" step="10"></td>`;
        b.appendChild(tr);
    }
}
function applyLimits() {
    for (let j = 0; j < NJ; j++) {
        const v = $(`v${j}`).value, a = $(`a${j}`).value, k = $(`k${j}`).value;
        if (v) cmd(`vel ${j + 1} ${v}`);
        if (a) cmd(`acc ${j + 1} ${a}`);
        if (k) cmd(`jerk ${j + 1} ${k}`);
    }
    toast('Limits applied', 'good');
}

// ------------------------------------------------------------ segments/tabs
function segment(el, items, isOn, onPick) {
    el.innerHTML = '';
    items.forEach(([val, label, title]) => {
        const b = document.createElement('button');
        b.className = 'btn sm' + (isOn(val) ? ' on' : '');
        b.textContent = label;
        if (title) b.title = title;
        b.onclick = () => onPick(val);
        el.appendChild(b);
    });
}

function showTab(name) {
    if (!TABS.some(([v]) => v === name)) name = 'jog';
    S.tab = name;
    document.querySelectorAll('.pane').forEach(p => p.classList.toggle('on', p.dataset.pane === name));
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('on', t.dataset.tab === name));
}
function buildTabs() {
    const nav = $('tabs'); nav.innerHTML = '';
    TABS.forEach(([val, label]) => {
        const b = document.createElement('button');
        b.className = 'tab'; b.dataset.tab = val; b.textContent = label;
        b.onclick = () => showTab(val);
        nav.appendChild(b);
    });
}

function renderModes() {
    segment($('modeBar'), [['step', 'Step'], ['hold', 'Hold']], v => v === S.jogMode, v => {
        stopEverything(); S.jogMode = v; renderModes();
    });
}
function renderFrameBars() {
    const items = [['base', 'Base'], ['tool', 'Tool']];
    const pick = v => { stopAllCartJogs(); S.cartFrame = v; viewer.setFrame(v); cmd(`cartframe ${v}`); renderFrameBars(); };
    segment($('frameBar'), items, v => v === S.cartFrame, pick);
    segment($('frameBarVp'), items, v => v === S.cartFrame, pick);
    $('linUnit').textContent = S.cartFrame === 'tool' ? '· tool axes' : '· base axes';
    $('angUnit').textContent = S.cartFrame === 'tool' ? '· tool axes' : '· base axes';
}
function renderIncrements() {
    segment($('incBar'), (config.increments || []).map(v => [v, v + '°']), v => v === S.selInc,
        v => { S.selInc = v; renderIncrements(); });
    segment($('cartLinIncBar'), [0.001, 0.005, 0.01, 0.05].map(v => [v, `${v * 1000}mm`]),
        v => v === S.cartLinStep, v => { S.cartLinStep = v; renderIncrements(); });
    segment($('cartAngIncBar'), [1, 5, 15, 45].map(v => [v, v + '°']),
        v => v === S.cartAngStep, v => { S.cartAngStep = v; renderIncrements(); });
}
function renderViewBar() {
    segment($('viewBar'), [['iso', 'Iso'], ['front', 'Front'], ['side', 'Side'], ['top', 'Top']],
        () => false, v => viewer.setView(v));
}

// ---------------------------------------------------------------- plan mode
let planning = false, planPose = null;
function togglePlan() {
    if (!viewer.setPlanning(!planning)) { toast('Model still loading', 'warn'); return; }
    planning = !planning;
    $('btnPlan').classList.toggle('on', planning);
    $('planCard').hidden = !planning;
    if (planning) { stopEverything(); renderPlanModeBar(); }
}
function renderPlanModeBar() {
    segment($('planModeBar'), [['translate', 'Move'], ['rotate', 'Rotate']],
        v => v === (viewer.gizmo.mode || 'translate'),
        v => { viewer.setPlanMode(v); renderPlanModeBar(); });
}
function renderPlan(p) {
    planPose = p;
    $('planReach').textContent = p.reachable ? 'reachable' : 'unreachable';
    $('planReach').className = 'badge ' + (p.reachable ? 'good' : 'bad');
    $('btnPlanGoL').disabled = $('btnPlanGoJ').disabled = !p.reachable;
    $('planReadout').innerHTML =
        `<b>X</b><span class="v">${p.pos[0].toFixed(4)}</span><b>RX</b><span class="v">${(p.rpy[0] * rad2deg).toFixed(1)}°</span>` +
        `<b>Y</b><span class="v">${p.pos[1].toFixed(4)}</span><b>RY</b><span class="v">${(p.rpy[1] * rad2deg).toFixed(1)}°</span>` +
        `<b>Z</b><span class="v">${p.pos[2].toFixed(4)}</span><b>RZ</b><span class="v">${(p.rpy[2] * rad2deg).toFixed(1)}°</span>`;
}
function planCmd(verb) {
    if (!planPose || !planPose.reachable) return;
    cmd(`${verb} ${[...planPose.pos, ...planPose.rpy].map(v => v.toFixed(5)).join(' ')}`);
    toast(verb === 'movel' ? 'Straight-line move sent' : 'Joint move sent');
}

// -------------------------------------------------------------------- poses
function renderPoses() {
    const list = $('poseList'); list.innerHTML = '';
    const sel = $('seqPose'); sel.innerHTML = '';
    Object.entries(config.poses).forEach(([name, a]) => {
        const it = document.createElement('div'); it.className = 'item';
        it.innerHTML = `<span class="nm"></span>
            <button class="btn sm accent" data-go>Go</button>
            <button class="btn sm bad" data-del>✕</button>`;
        it.querySelector('.nm').textContent = name;               // name is user data: never innerHTML
        it.querySelector('[data-go]').onclick = () => cmd(a.map(x => (+x).toFixed(3)).join(' '));
        it.querySelector('[data-del]').onclick = () => send({ type: 'delete_pose', name });
        list.appendChild(it);
        const o = document.createElement('option'); o.value = name; o.textContent = name; sel.appendChild(o);
    });
}
function savePose() {
    const name = $('poseName').value.trim();
    if (!name) return;
    send({ type: 'save_pose', name, angles: state.tgt.map(Number) });
    $('poseName').value = '';
}

// ----------------------------------------------------------------- sequences
function renderSteps() {
    const l = $('stepList'); l.innerHTML = '';
    curSteps.forEach((s, i) => {
        const it = document.createElement('div'); it.className = 'item';
        it.innerHTML = `<span class="mut">${i + 1}.</span><span class="nm"></span>
            <span class="mut">${s.dwell}s</span><button class="btn sm bad" data-del>✕</button>`;
        it.querySelector('.nm').textContent = s.pose;
        it.querySelector('[data-del]').onclick = () => { curSteps.splice(i, 1); renderSteps(); };
        l.appendChild(it);
    });
}
function renderSeqs() {
    const l = $('seqList'); l.innerHTML = '';
    Object.entries(config.sequences).forEach(([name, steps]) => {
        const it = document.createElement('div'); it.className = 'item';
        it.innerHTML = `<span class="nm"></span><span class="mut">${steps.length} steps</span>
            <button class="btn sm accent" data-play>▶</button><button class="btn sm bad" data-del>✕</button>`;
        it.querySelector('.nm').textContent = name;
        it.querySelector('[data-play]').onclick = () => send({ type: 'play', steps: stepsToAngles(steps) });
        it.querySelector('[data-del]').onclick = () => send({ type: 'delete_seq', name });
        l.appendChild(it);
    });
}
const stepsToAngles = (steps) => steps
    .map(s => ({ angles: (config.poses[s.pose] || []).map(Number), dwell: s.dwell }))
    .filter(s => s.angles.length === NJ);

function renderAll() {
    renderModes(); renderFrameBars(); renderIncrements();
    renderPoses(); renderSeqs(); renderSteps(); renderMotors();
}

// ------------------------------------------------------------------- actions
const actions = {
    sync: () => cmd('sync'),
    home: () => cmd('home'),
    ready: () => cmd('ready'),
    stop: () => { stopEverything(); cmd('stop'); },
    rehome: () => { if (confirm('Rehome all joints using the limit switches?')) cmd('rehome'); },
    applyLimits, savePose, saveSettings, resetSettings,
    addIncrement: () => {
        const v = parseFloat($('incCustom').value);
        if (!isFinite(v) || v <= 0) return;
        send({ type: 'set_increments', values: [...new Set([...(config.increments || []), v])].sort((a, b) => a - b) });
        S.selInc = v;
        $('incCustom').value = '';
    },
    addStep: () => {
        const pose = $('seqPose').value;
        if (!pose) return;
        curSteps.push({ pose, dwell: parseFloat($('seqDwell').value) || 1 });
        renderSteps();
    },
    saveSeq: () => {
        const name = $('seqName').value.trim();
        if (!name || !curSteps.length) return;
        send({ type: 'save_seq', name, steps: curSteps });
        $('seqName').value = '';
    },
    play: () => { if (curSteps.length) send({ type: 'play', steps: stepsToAngles(curSteps) }); },
    stopPlay: () => { send({ type: 'stop_play' }); stopEverything(); },
};
document.querySelectorAll('[data-act]').forEach(el => {
    el.addEventListener('click', () => actions[el.dataset.act]?.());
});

$('btnTrail').onclick = () => {
    S.trail = !S.trail;
    $('btnTrail').classList.toggle('on', S.trail);
    viewer.setTrail(S.trail);
};
$('btnTrailClear').onclick = () => viewer.clearTrail();
$('btnPlan').onclick = togglePlan;
$('btnPlanGoL').onclick = () => planCmd('movel');
$('btnPlanGoJ').onclick = () => planCmd('move');
$('btnPlanReset').onclick = () => viewer.resetPlan();

$('jogSpeed').onchange = (e) => { S.jogSpeed = +e.target.value || DEFAULTS.jogSpeed; };
$('cartLinSpeed').onchange = (e) => { S.cartLinSpeed = +e.target.value || DEFAULTS.cartLinSpeed; };
$('cartAngSpeed').onchange = (e) => { S.cartAngSpeed = +e.target.value || DEFAULTS.cartAngSpeed; };

// Spacebar is a hard stop from anywhere except while typing in a field.
document.addEventListener('keydown', (e) => {
    if (e.code !== 'Space') return;
    const t = e.target;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return;
    e.preventDefault();
    actions.stop();
    toast('STOP', 'bad', 1500);
});

// ---------------------------------------------------------------------- init
buildTabs();
buildCartPad('cartLin', [0, 1, 2]);
buildCartPad('cartAng', [3, 4, 5]);
buildLimits();
buildJoints();
renderViewBar();
renderAll();
showTab(S.tab);
connect();
