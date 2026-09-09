import { RobotViewer, AXIS_COLORS } from './viewer.js';

const NJ = 6;
const CART_AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const JOG_REFRESH_MS = 60;          // must stay under the controller's 200 ms dead-man
const TABS = [['jog', 'Jog'], ['joints', 'Joints'], ['points', 'Points'],
              ['program', 'Program'], ['setup', 'Setup']];
const NODE_KINDS = [['move', 'Move'], ['gripper', 'Gripper'], ['wait', 'Wait'],
                    ['speed', 'Speed'], ['loop', 'Loop'], ['call', 'Call'], ['comment', 'Note']];

const $ = (id) => document.getElementById(id);
const rad2deg = 180 / Math.PI;
const axColor = (a) => '#' + AXIS_COLORS[a % 3].toString(16).padStart(6, '0');

const DEFAULTS = {
    jogMode: 'hold', cartFrame: 'base',
    jogSpeed: 20, cartLinSpeed: 0.05, cartAngSpeed: 15,
    selInc: 10, cartLinStep: 0.01, cartAngStep: 5,
    trail: true, tab: 'jog', grip: 'default',
};
let S = { ...DEFAULTS };
const GRIP_MAX = 140;
const DEFAULT_GRIP = { open: 0, close: GRIP_MAX };

let state = {
    pos: Array(NJ).fill(0), tgt: Array(NJ).fill(0), tcp: Array(6).fill(0),
    vmax: Array(NJ).fill(0), amax: Array(NJ).fill(0), jmax: Array(NJ).fill(0),
    enabled: Array(NJ).fill(1), sigma: 1, frame: 'base',
};
let config = { poses: {}, programs: {}, increments: [5, 10, 20], settings: {}, grips: {} };
let jointLimits = null;
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
        else if (m.type === 'prog') onProg(m);
        else if (m.type === 'prog_done') toast('Program finished', 'good');
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
// The server re-broadcasts the whole config on every save (teach a waypoint,
// save a program, save a grip...), not just on connect. Re-adopting the
// persisted settings blob on every one of those would snap the UI back to
// whatever tab/jog-mode was current the last time "Save settings" was
// pressed -- so only the first config message seeds live session state (S);
// later ones just refresh the data (poses/programs/grips/increments).
let configLoaded = false;
function onConfig(m) {
    config = { increments: [5, 10, 20], settings: {}, grips: {}, programs: {}, ...m };
    if (!configLoaded) {
        S = { ...DEFAULTS, ...(config.settings || {}) };
        applySettings();
        configLoaded = true;
    }
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

// The controller reports which URDF it was built against, so the viewer can
// never end up showing a different model than the kinematics is solving.
let urdfLoaded = null;
function loadModel(name) {
    if (urdfLoaded === name) return;
    urdfLoaded = name;
    viewer.load(`/assets/${name}`).then(() => {
        const msg = $('loadMsg');
        if (msg) msg.remove();
        jointLimits = viewer.jointLimits();
        buildJoints();
        viewer.setFrame(S.cartFrame);
    }).catch(err => {
        const msg = $('loadMsg');
        if (msg) msg.textContent = 'failed to load model: ' + err;
        console.error(err);
    });
}

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
let speedBusy = 0;      // don't fight the user's finger on the override slider
function onState(m) {
    state = { ...state, ...m, enabled: Array.isArray(m.enabled) ? m.enabled : state.enabled };
    if (m.urdf) loadModel(m.urdf);
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
    if (typeof m.sim === 'boolean') renderSim(m.sim, m.can_sim_off !== false);
    if (typeof m.grip === 'number') renderGrip(m.grip, m.grip_ack);
    if (typeof m.speed === 'number' && Date.now() - speedBusy > 700) {
        $('speedOv').value = m.speed;
        $('speedOvVal').textContent = Math.round(m.speed) + '%';
    }

    if (typeof m.sigma === 'number' && m.sigma < 0.02 && Date.now() - lastSigmaWarn > 5000) {
        lastSigmaWarn = Date.now();
        toast(`Near singularity (σmin ${m.sigma.toFixed(4)}) — Cartesian motion may be blocked`, 'warn', 4000);
    }
    renderMotors();
}
function setIfIdle(id, v) { const el = $(id); if (el && document.activeElement !== el) el.value = v; }

// The controller owns this flag; the button only mirrors it and asks for a change.
let simOn = null;
function renderSim(on, canLeave) {
    const b = $('btnSim');
    b.hidden = false;
    b.classList.toggle('on', on);
    b.textContent = on ? 'SIM' : 'LIVE';
    b.disabled = on && !canLeave;
    b.title = b.disabled
        ? 'No serial port open — restart with a real port to leave the simulator'
        : (on ? 'Simulated ESP32 — click to switch to hardware'
              : 'Driving real hardware — click to switch to the simulator');
    document.body.classList.toggle('simulating', on);
    if (simOn !== null && simOn !== on) toast(on ? 'Simulator ON' : 'Now driving hardware', on ? 'warn' : 'good');
    simOn = on;
}

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

// ------------------------------------------------------------------ gripper
// The controller only knows raw 0..140. Named presets ("how far to close for
// THIS part") are task metadata and live in poses.json with the poses.
const grip = () => config.grips?.[S.grip] || DEFAULT_GRIP;
let gripBusy = 0;

function sendGrip(v) {
    v = Math.max(0, Math.min(GRIP_MAX, Math.round(v)));
    gripBusy = Date.now();
    $('gripSlider').value = v;
    $('gripVal').textContent = v;
    cmd(`gripper ${v}`);
}

function renderGrip(cmdPos, ackPos) {
    viewer.setGripper(typeof ackPos === 'number' ? ackPos : cmdPos, GRIP_MAX);
    if (Date.now() - gripBusy > 700) {
        $('gripSlider').value = cmdPos;
        $('gripVal').textContent = cmdPos;
    }
    const el = $('gripAck');
    if (typeof ackPos !== 'number') { el.textContent = ''; el.className = 'badge'; return; }
    // The ESP echoes what ESP-NOW actually delivered; a lasting mismatch means
    // the gripper ESP is unreachable, not just mid-travel.
    const ok = ackPos === cmdPos;
    el.textContent = ok ? `at ${ackPos}` : `sending ${cmdPos}\u2192`;
    el.className = 'badge ' + (ok ? 'good' : 'warn');
}

function renderGrips() {
    const names = Object.keys(config.grips || {});
    if (!names.includes(S.grip)) S.grip = names[0] || 'default';
    segment($('gripPresetBar'), names.length ? names.map(n => [n, n]) : [['default', 'default']],
        v => v === S.grip, v => { S.grip = v; renderGrips(); });
    const g = grip();
    $('btnGripOpen').title = `open to ${g.open}`;
    $('btnGripClose').title = `close to ${g.close}`;
}

function saveGrip() {
    const name = $('gripName').value.trim() || S.grip;
    if (!name) return;
    // Whatever the slider is at right now IS the taught closing point.
    send({ type: 'save_grip', name, open: grip().open, close: +$('gripSlider').value });
    S.grip = name;
    $('gripName').value = '';
    toast(`Grip "${name}" closes at ${$('gripSlider').value}`, 'good');
}

// ------------------------------------------------------------------- motors
function renderMotors() {    const bar = $('motorBar'); bar.innerHTML = '';
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

// ---------------------------------------------------------------- waypoints
// Waypoints are stored as JOINT ANGLES, not a pose: one TCP pose is reachable in
// up to 8 joint configurations, so replaying a pose alone can flip the elbow or
// wrist. The pose is saved alongside purely for display, and comes from the
// controller's own FK so it can't disagree with the kinematics.
const poseAngles = (p) => (Array.isArray(p) ? p : p?.angles) || [];
const posePose = (p) => (Array.isArray(p) ? null : p?.pose) || null;
const wpNames = () => Object.keys(config.poses);
function wpAngles(name) {
    const a = poseAngles(config.poses[name]);
    return a.length === NJ ? a.map(Number) : null;
}

function renderWaypoints() {
    const list = $('wpList'); list.innerHTML = '';
    Object.entries(config.poses).forEach(([name, p]) => {
        const a = poseAngles(p), tcp = posePose(p);
        if (a.length !== NJ) return;
        const j = a.map(x => (+x).toFixed(3)).join(' ');
        const it = document.createElement('div'); it.className = 'item';
        it.innerHTML = `<span class="nm"></span>
            <span class="mut sub"></span>
            <button class="btn sm" data-goj title="joint move to here">J</button>
            <button class="btn sm" data-gol title="straight-line move to here">L</button>
            <button class="btn sm" data-up title="re-teach at the current position">⟳</button>
            <button class="btn sm bad" data-del>✕</button>`;
        it.querySelector('.nm').textContent = name;               // name is user data: never innerHTML
        it.querySelector('.sub').textContent = tcp
            ? `${tcp[0].toFixed(3)} ${tcp[1].toFixed(3)} ${tcp[2].toFixed(3)}`
            : a.map(x => (+x).toFixed(0)).join(',');
        it.querySelector('[data-goj]').onclick = () => cmd(j);
        it.querySelector('[data-gol]').onclick = () => cmd('movel q ' + j);
        it.querySelector('[data-up]').onclick = () => teachWp(name);
        it.querySelector('[data-del]').onclick = () => {
            if (confirm(`Delete waypoint "${name}"? Programs using it will refuse to run.`))
                send({ type: 'delete_pose', name });
        };
        list.appendChild(it);
    });
}

function teachWp(name) {
    name = (name || $('wpName').value).trim();
    if (!name) { toast('Name the waypoint first', 'warn'); return null; }
    // tgt, not pos: the settled planner target carries no encoder noise or
    // following error, and it is exactly what gets commanded on replay.
    send({ type: 'save_pose', name, angles: state.tgt.map(Number), pose: state.tcp.map(Number) });
    $('wpName').value = '';
    toast(`Taught "${name}"`, 'good', 1800);
    return name;
}
function autoWpName() {
    let i = 1;
    while (config.poses[`P${i}`]) i++;
    return `P${i}`;
}

// ------------------------------------------------------------------ program
// A program is a tree of typed steps, not a flat pose list: loops need a body,
// and moves reference waypoints by NAME so re-teaching a point updates every
// program that uses it.
let progName = '';
let progNodes = [];
let selNode = null;      // the step the editor has selected
let runNode = null;      // the step the interpreter is on
let runCounters = {};
let progDirty = false;

let uidN = 0;
const uid = () => `n${Date.now().toString(36)}${(uidN++).toString(36)}`;

function newNode(type) {
    const b = { id: uid(), type };
    switch (type) {
        case 'move':    return { ...b, motion: 'j', wp: wpNames()[0] || '', dwell: 0 };
        case 'gripper': return { ...b, value: grip().close, settle: 0.5 };
        case 'wait':    return { ...b, seconds: 1 };
        case 'speed':   return { ...b, percent: 50 };
        case 'loop':    return { ...b, mode: 'count', count: 3, body: [] };
        case 'call':    return { ...b, name: Object.keys(config.programs || {})[0] || '' };
        default:        return { ...b, text: '' };
    }
}
const cloneNode = (n) => ({ ...n, id: uid(), ...(n.body ? { body: n.body.map(cloneNode) } : {}) });

function flatten(nodes, depth = 0, out = []) {
    for (const n of nodes) {
        out.push({ n, depth });
        if (n.type === 'loop') flatten(n.body || [], depth + 1, out);
    }
    return out;
}
// The array a step lives in, its index there, and the loop enclosing it.
function locate(id, nodes = progNodes, parent = null) {
    for (let i = 0; i < nodes.length; i++) {
        if (nodes[i].id === id) return { arr: nodes, i, parent };
        if (nodes[i].type === 'loop') {
            const r = locate(id, nodes[i].body || [], nodes[i]);
            if (r) return r;
        }
    }
    return null;
}
function ensureIds(nodes) {
    for (const n of nodes) {
        if (!n.id) n.id = uid();
        if (n.body) ensureIds(n.body);
    }
}

function nodeLabel(n) {
    switch (n.type) {
        case 'move':    return [`move-${n.motion}`, `MOVE ${(n.motion || 'j').toUpperCase()}`,
                                (n.wp || '— no waypoint —') + (n.dwell ? ` · dwell ${n.dwell}s` : '')];
        case 'gripper': return ['grip', 'GRIP', `to ${n.value}`];
        case 'wait':    return ['', 'WAIT', `${n.seconds} s`];
        case 'speed':   return ['', 'SPEED', `${n.percent}%`];
        case 'loop':    return ['loop', 'LOOP', n.mode === 'forever' ? 'forever' : `${n.count} ×`];
        case 'call':    return ['', 'CALL', n.name || '— no program —'];
        default:        return ['', 'NOTE', n.text || ''];
    }
}

function renderTree() {
    const tree = $('progTree'); tree.innerHTML = '';
    flatten(progNodes).forEach(({ n, depth }, i) => {
        const [cls, kind, txt] = nodeLabel(n);
        const row = document.createElement('div');
        row.className = 'node'
            + (n.id === selNode ? ' sel' : '')
            + (n.id === runNode ? ' run' : '')
            + (n.type === 'comment' ? ' comment' : '');
        row.innerHTML = `<span class="n">${i + 1}</span>`
            + '<span class="rail"></span>'.repeat(depth)
            + `<span class="kind ${cls}"></span><span class="txt"></span>`
            + (runCounters[n.id] ? '<span class="badge good ctr"></span>' : '');
        row.querySelector('.kind').textContent = kind;
        row.querySelector('.txt').textContent = txt;
        const ctr = row.querySelector('.ctr');
        if (ctr) ctr.textContent = runCounters[n.id];
        row.onclick = () => { selNode = n.id; renderTree(); renderInspector(); };
        tree.appendChild(row);
    });
    if (runNode) tree.querySelector('.node.run')?.scrollIntoView({ block: 'nearest' });
    document.querySelector('[data-act="progSave"]')?.classList.toggle('accent', progDirty);
}

// ------------------------------------------------------- structural editing
function insertNode(node) {
    const loc = selNode && locate(selNode);
    if (!loc) progNodes.push(node);
    else if (loc.arr[loc.i].type === 'loop') (loc.arr[loc.i].body ||= []).push(node);
    else loc.arr.splice(loc.i + 1, 0, node);
    selNode = node.id;
    touchProgram();
}
function touchProgram() { progDirty = true; renderTree(); renderInspector(); }

const nodeOps = {
    nodeUp: () => {
        const l = locate(selNode);
        if (!l || l.i === 0) return;
        [l.arr[l.i - 1], l.arr[l.i]] = [l.arr[l.i], l.arr[l.i - 1]];
        touchProgram();
    },
    nodeDown: () => {
        const l = locate(selNode);
        if (!l || l.i >= l.arr.length - 1) return;
        [l.arr[l.i + 1], l.arr[l.i]] = [l.arr[l.i], l.arr[l.i + 1]];
        touchProgram();
    },
    nodeIn: () => {
        const l = locate(selNode);
        const prev = l && l.i > 0 ? l.arr[l.i - 1] : null;
        if (!prev || prev.type !== 'loop') { toast('Put a Loop directly above this step first', 'warn'); return; }
        (prev.body ||= []).push(l.arr.splice(l.i, 1)[0]);
        touchProgram();
    },
    nodeOut: () => {
        const l = locate(selNode);
        if (!l || !l.parent) return;
        const p = locate(l.parent.id);
        p.arr.splice(p.i + 1, 0, l.arr.splice(l.i, 1)[0]);
        touchProgram();
    },
    nodeDup: () => {
        const l = locate(selNode);
        if (!l) return;
        const copy = cloneNode(l.arr[l.i]);
        l.arr.splice(l.i + 1, 0, copy);
        selNode = copy.id;
        touchProgram();
    },
    nodeDel: () => {
        const l = locate(selNode);
        if (!l) return;
        l.arr.splice(l.i, 1);
        selNode = null;
        touchProgram();
    },
};

// ----------------------------------------------------------- step inspector
function field(label, ...els) {
    const r = document.createElement('div'); r.className = 'row';
    const l = document.createElement('label'); l.textContent = label;
    r.append(l, ...els);
    return r;
}
function selectOf(values, value, onPick) {
    const s = document.createElement('select');
    values.forEach(v => {
        const o = document.createElement('option');
        o.value = v; o.textContent = v; s.appendChild(o);
    });
    s.value = value;
    s.onchange = () => onPick(s.value);
    return s;
}
function numberOf(value, step, min, onSet) {
    const el = document.createElement('input');
    el.type = 'number'; el.className = 'num'; el.value = value; el.step = step;
    if (min !== null) el.min = min;
    el.onchange = () => onSet(parseFloat(el.value) || 0);
    return el;
}

function renderInspector() {
    const box = $('inspector'); box.innerHTML = '';
    const loc = selNode && locate(selNode);
    if (!loc) return;
    const n = loc.arr[loc.i];
    const set = (k, v) => { n[k] = v; touchProgram(); };

    if (n.type === 'move') {
        const bar = document.createElement('div'); bar.className = 'seg grow';
        segment(bar, [['j', 'Move J', 'joint-interpolated — fastest, path is not straight'],
                      ['l', 'Move L', 'straight line through Cartesian space']],
            v => v === n.motion, v => set('motion', v));
        box.append(field('Motion', bar));

        const names = wpNames();
        box.append(field('Waypoint', selectOf(names.length ? names : [''], n.wp, v => set('wp', v))));
        box.append(field('Dwell s', numberOf(n.dwell || 0, 0.1, 0, v => set('dwell', v))));

        const go = document.createElement('button');
        go.className = 'btn sm'; go.textContent = 'Preview this move';
        go.onclick = () => {
            const a = wpAngles(n.wp);
            if (!a) { toast(`Waypoint "${n.wp}" is missing`, 'bad'); return; }
            const j = a.map(x => x.toFixed(3)).join(' ');
            cmd(n.motion === 'l' ? `movel q ${j}` : j);
        };
        box.append(field('', go));

    } else if (n.type === 'gripper') {
        const sl = document.createElement('input');
        sl.type = 'range'; sl.min = 0; sl.max = GRIP_MAX; sl.step = 1; sl.value = n.value;
        sl.style.flex = '1'; sl.style.minWidth = '0';
        const out = document.createElement('span');
        out.className = 'val'; out.textContent = n.value;
        sl.oninput = () => { out.textContent = sl.value; };
        sl.onchange = () => set('value', +sl.value);
        box.append(field('Position', sl, out));

        const bar = document.createElement('div'); bar.className = 'seg grow';
        segment(bar, [['open', 'Open'], ['close', 'Close']], () => false, v => set('value', grip()[v]));
        box.append(field('From preset', bar));
        box.append(field('Settle s', numberOf(n.settle ?? 0.5, 0.1, 0, v => set('settle', v))));

    } else if (n.type === 'wait') {
        box.append(field('Seconds', numberOf(n.seconds, 0.1, 0, v => set('seconds', v))));

    } else if (n.type === 'speed') {
        box.append(field('Percent', numberOf(n.percent, 5, 5,
            v => set('percent', Math.max(5, Math.min(100, Math.round(v)))))));

    } else if (n.type === 'loop') {
        const bar = document.createElement('div'); bar.className = 'seg grow';
        segment(bar, [['count', 'Count'], ['forever', 'Forever']], v => v === n.mode, v => set('mode', v));
        box.append(field('Mode', bar));
        if (n.mode !== 'forever')
            box.append(field('Repeats', numberOf(n.count, 1, 1, v => set('count', Math.max(1, Math.round(v))))));

    } else if (n.type === 'call') {
        const names = Object.keys(config.programs || {}).filter(x => x !== progName);
        box.append(field('Program', selectOf(names.length ? names : [''], n.name, v => set('name', v))));

    } else if (n.type === 'comment') {
        const t = document.createElement('input');
        t.type = 'text'; t.value = n.text || ''; t.placeholder = 'note';
        t.onchange = () => set('text', t.value);
        box.append(field('Text', t));
    }
}

// -------------------------------------------------------- program lifecycle
function renderProgList() {
    const sel = $('progSel'); sel.innerHTML = '';
    if (!progName) {
        const o = document.createElement('option');
        o.value = ''; o.textContent = '— untitled —';
        sel.appendChild(o);
    }
    Object.keys(config.programs || {}).forEach(nm => {
        const o = document.createElement('option');
        o.value = nm; o.textContent = nm;
        sel.appendChild(o);
    });
    sel.value = progName;
}
function loadProgram(name) {
    if (progDirty && !confirm('Discard unsaved changes to this program?')) { renderProgList(); return; }
    progName = name;
    progNodes = JSON.parse(JSON.stringify((config.programs || {})[name] || []));
    ensureIds(progNodes);
    selNode = null; progDirty = false;
    renderProgList(); renderTree(); renderInspector();
}

// Everything the interpreter would otherwise only discover mid-run.
function progIssues() {
    const out = [];
    flatten(progNodes).forEach(({ n }) => {
        if (n.type === 'move' && !wpAngles(n.wp)) out.push(`Move: waypoint "${n.wp || '—'}" is missing`);
        else if (n.type === 'call' && n.name === progName) out.push(`Call: "${n.name}" calls itself`);
        else if (n.type === 'call' && !(config.programs || {})[n.name]) out.push(`Call: program "${n.name || '—'}" is missing`);
    });
    return out;
}
function startProgram(step) {
    if (!progNodes.length) { toast('Program is empty', 'warn'); return; }
    const bad = progIssues();
    if (bad.length) { toast(bad[0], 'bad', 5000); return; }
    send({ type: 'run', nodes: progNodes, step });
}

let progRunning = false, progPaused = false;
function onProg(m) {
    progRunning = !!m.running;
    progPaused = !!m.paused;
    runNode = m.running ? m.node : null;
    runCounters = m.counters || {};
    const el = $('progStatus');
    if (m.error) { el.textContent = m.error; el.className = 'badge bad'; toast(m.error, 'bad', 6000); }
    else if (!m.running) { el.textContent = 'idle'; el.className = 'badge'; }
    else if (m.paused) { el.textContent = 'paused'; el.className = 'badge warn'; }
    else { el.textContent = 'running'; el.className = 'badge good'; }
    renderTree();
}

function renderPalette() {
    const bar = $('palette'); bar.innerHTML = '';
    const add = (label, title, fn) => {
        const b = document.createElement('button');
        b.className = 'btn sm'; b.textContent = label; b.title = title;
        b.onclick = fn;
        bar.appendChild(b);
    };
    add('＋ Here', 'Teach the current position as a new waypoint, then move to it', () => {
        const name = teachWp(autoWpName());
        if (name) insertNode({ ...newNode('move'), wp: name });
    });
    NODE_KINDS.forEach(([type, label]) => add(label, `insert a ${label} step`, () => insertNode(newNode(type))));
}

function renderAll() {
    renderModes(); renderFrameBars(); renderIncrements();
    renderWaypoints(); renderProgList(); renderPalette();
    renderTree(); renderInspector(); renderMotors(); renderGrips();
}

// ------------------------------------------------------------------- actions
const actions = {
    sync: () => cmd('sync'),
    home: () => cmd('home'),
    ready: () => cmd('ready'),
    stop: () => { stopEverything(); send({ type: 'prog_ctl', action: 'stop' }); cmd('stop'); },
    rehome: () => { if (confirm('Rehome all joints using the limit switches?')) cmd('rehome'); },
    applyLimits, saveSettings, resetSettings, saveGrip,
    teachWp: () => teachWp(),
    addIncrement: () => {
        const v = parseFloat($('incCustom').value);
        if (!isFinite(v) || v <= 0) return;
        send({ type: 'set_increments', values: [...new Set([...(config.increments || []), v])].sort((a, b) => a - b) });
        S.selInc = v;
        $('incCustom').value = '';
    },
    progNew: () => {
        if (progDirty && !confirm('Discard unsaved changes to this program?')) return;
        progName = ''; progNodes = []; selNode = null; progDirty = false;
        renderProgList(); renderTree(); renderInspector();
    },
    progSave: () => {
        const name = progName || (prompt('Program name') || '').trim();
        if (!name) return;
        progName = name;
        send({ type: 'save_program', name, nodes: progNodes });
        progDirty = false;
        toast(`Saved "${name}"`, 'good', 1800);
    },
    progDelete: () => {
        if (!progName || !confirm(`Delete program "${progName}"?`)) return;
        send({ type: 'delete_program', name: progName });
        progDirty = false;
        actions.progNew();
    },
    progRun: () => startProgram(false),
    // Stepping from a standing start also has to *begin* the program.
    progStep: () => progRunning ? send({ type: 'prog_ctl', action: 'step' }) : startProgram(true),
    progPause: () => send({ type: 'prog_ctl', action: progPaused ? 'resume' : 'pause' }),
    progStop: () => send({ type: 'prog_ctl', action: 'stop' }),
    ...nodeOps,
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
$('btnSim').onclick = () => {
    if (simOn && !confirm('Leave the simulator and drive the real arm?')) return;
    stopEverything();
    cmd(`sim ${simOn ? 'off' : 'on'}`);
};
$('btnPlan').onclick = togglePlan;
$('btnPlanGoL').onclick = () => planCmd('movel');
$('btnPlanGoJ').onclick = () => planCmd('move');
$('btnPlanReset').onclick = () => viewer.resetPlan();

$('btnGripOpen').onclick = () => sendGrip(grip().open);
$('btnGripClose').onclick = () => sendGrip(grip().close);
$('gripSlider').oninput = (e) => sendGrip(+e.target.value);

$('progSel').onchange = (e) => loadProgram(e.target.value);
$('speedOv').oninput = (e) => {
    $('speedOvVal').textContent = e.target.value + '%';
    speedBusy = Date.now();
    cmd(`speed ${e.target.value}`);
};

$('jogSpeed').onchange = (e) => { S.jogSpeed = +e.target.value || DEFAULTS.jogSpeed; };$('cartLinSpeed').onchange = (e) => { S.cartLinSpeed = +e.target.value || DEFAULTS.cartLinSpeed; };
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
