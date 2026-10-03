import { create } from 'zustand';
import { toast } from 'sonner';

export const NJ = 6;
export const CART_AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
export const GRIP_MAX = 140;
export const R2D = 180 / Math.PI;
const JOG_REFRESH_MS = 60;          // must stay under the controller's jog dead-man
const FALLBACK_URDF = 'parol6-PGripper.urdf';

// Session settings, persisted to poses.json by "Save settings".
export const DEFAULTS = {
    jogMode: 'hold', cartFrame: 'base',
    jogSpeed: 20, cartLinSpeed: 0.05, cartAngSpeed: 15,
    selInc: 10, cartLinStep: 0.01, cartAngStep: 5,
    trail: true, tab: 'jog', grip: 'default',
};
// Per-device preferences (a phone and a desktop want different quality).
const PREFS = { theme: 'dark', quality: 'balanced', tcp: 'a', autoMotion: 'j', panel: true };
const EMPTY_CONFIG = { poses: {}, programs: {}, increments: [5, 10, 20], settings: {}, grips: {} };

function loadPrefs() {
    try { return { ...PREFS, ...JSON.parse(localStorage.getItem('parol.prefs')) }; } catch { return { ...PREFS }; }
}

const zeros = (n) => new Array(n).fill(0);

// High-rate controller state, mutated in place and read by the 3D scene every
// frame. Kept out of React so 50 Hz joint updates don't re-render the HUD.
export const live = { q: zeros(NJ), tcp: zeros(6), sigma: 1, busy: false, grip: 0 };

let invalidate = () => {};
export const setInvalidate = (fn) => { invalidate = fn; };
// Encoder noise must not keep the GPU busy: ignore sub-0.001° changes.
const moved = (a, b) => a.some((v, i) => Math.abs(v - b[i]) > 1e-3);

export const useStore = create(() => ({
    conn: 'connecting',
    urdf: null,
    // ~15 Hz snapshot of the controller state for the panels.
    st: {
        pos: zeros(NJ), tgt: zeros(NJ), enabled: new Array(NJ).fill(1), speed: 100,
        sim: null, canSimOff: true, grip: 0, gripAck: null,
        vmax: zeros(NJ), amax: zeros(NJ), jmax: zeros(NJ), frame: 'base',
    },
    tcp: zeros(6),
    sigma: 1,
    config: EMPTY_CONFIG,
    S: { ...DEFAULTS },
    prefs: loadPrefs(),
    prog: { running: false, paused: false, node: null, counters: {}, error: null },
    editor: { name: '', nodes: [], sel: null, dirty: false },
    limits: null,                  // [{lower, upper}] degrees, from the URDF
    mode: 'orbit',                 // orbit | tcp
    view: 'iso',
    viewNonce: 0,
    trailNonce: 0,
    plan: null,                    // ghost-target solution
    preview: 'j',                  // which path the target card is previewing
    hoverAxis: null,               // jog button under the pointer, mirrored in 3D
    hoverWp: null,
    jogHud: null,                  // { axis, rate } while a velocity jog is live
}));

export const setS = (patch) => useStore.setState((s) => ({ S: { ...s.S, ...patch } }));
export const setView = (view) => useStore.setState((s) => ({ view, viewNonce: s.viewNonce + 1 }));
export const clearTrail = () => useStore.setState((s) => ({ trailNonce: s.trailNonce + 1 }));
export function setPref(k, v) {
    const prefs = { ...useStore.getState().prefs, [k]: v };
    localStorage.setItem('parol.prefs', JSON.stringify(prefs));
    useStore.setState({ prefs });
}

// ----------------------------------------------------------------- transport
let ws = null;
let lastHud = 0;
let lastSigmaWarn = 0;
let configLoaded = false;

export function connect() {
    ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
    ws.onopen = () => useStore.setState({ conn: 'ok' });
    ws.onclose = () => {
        useStore.setState({ conn: 'down' });
        setTimeout(connect, 1000);
    };
    ws.onmessage = (e) => {
        const m = JSON.parse(e.data);
        if (m.type === 'state') onState(m);
        else if (m.type === 'config') onConfig(m);
        else if (m.type === 'prog') onProg(m);
        else if (m.type === 'prog_done') toast.success('Program finished');
    };
}

export function send(o) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(o));
    else toast.error('Controller unavailable', { id: 'unavailable' });
}
export const cmd = (line) => send({ type: 'cmd', line });

function onState(m) {
    const q0 = live.q, g0 = live.grip;
    if (Array.isArray(m.pos)) live.q = m.pos.map(Number);
    if (Array.isArray(m.tcp)) live.tcp = m.tcp.map(Number);
    if (typeof m.sigma === 'number') live.sigma = m.sigma;
    live.busy = !!m.busy;
    if (typeof m.grip === 'number') live.grip = typeof m.grip_ack === 'number' ? m.grip_ack : m.grip;
    if (moved(live.q, q0) || live.grip !== g0) invalidate();

    const s = useStore.getState();
    const patch = {};
    if (m.urdf && m.urdf !== s.urdf) patch.urdf = m.urdf;
    if (m.frame && configLoaded && m.frame !== s.S.cartFrame) patch.S = { ...s.S, cartFrame: m.frame };

    const simChanged = typeof m.sim === 'boolean' && m.sim !== s.st.sim;
    if (simChanged && s.st.sim !== null) {
        if (m.sim) toast.warning('Simulator on');
        else toast.success('Now driving hardware');
    }
    // Panels only need ~15 Hz; anything faster is wasted React work.
    const now = performance.now();
    if (now - lastHud > 66 || simChanged) {
        lastHud = now;
        const p = s.st;
        patch.tcp = live.tcp;
        patch.sigma = live.sigma;
        patch.st = {
            pos: m.pos ?? p.pos, tgt: m.tgt ?? p.tgt,
            enabled: Array.isArray(m.enabled) ? m.enabled : p.enabled,
            speed: typeof m.speed === 'number' ? m.speed : p.speed,
            sim: typeof m.sim === 'boolean' ? m.sim : p.sim,
            canSimOff: m.can_sim_off !== false,
            grip: typeof m.grip === 'number' ? m.grip : p.grip,
            gripAck: typeof m.grip_ack === 'number' ? m.grip_ack : null,
            vmax: m.vmax ?? p.vmax, amax: m.amax ?? p.amax, jmax: m.jmax ?? p.jmax,
            frame: m.frame ?? p.frame,
        };
    }
    if (live.sigma < 0.02 && Date.now() - lastSigmaWarn > 5000) {
        lastSigmaWarn = Date.now();
        toast.warning(`Near singularity (σmin ${live.sigma.toFixed(4)})`, {
            description: 'Cartesian motion may be blocked',
        });
    }
    if (Object.keys(patch).length) useStore.setState(patch);
}

// The server re-broadcasts the whole config on every save. Only the first one
// seeds the session settings, or every waypoint save would snap the UI back.
function onConfig(m) {
    const { type, ...c } = m;
    const config = { ...EMPTY_CONFIG, ...c };
    const patch = { config };
    if (!configLoaded) {
        configLoaded = true;
        const S = { ...DEFAULTS, ...(config.settings || {}) };
        patch.S = S;
        if (useStore.getState().st.frame !== S.cartFrame) cmd(`cartframe ${S.cartFrame}`);
    }
    useStore.setState(patch);
}

function onProg(m) {
    const prev = useStore.getState().prog;
    if (m.error && m.error !== prev.error) toast.error(m.error);
    useStore.setState({
        prog: { running: !!m.running, paused: !!m.paused, node: m.running ? m.node : null,
                counters: m.counters || {}, error: m.error || null },
    });
}

// Without a controller there is no state stream to name the URDF, so still
// show the arm rather than an empty stage.
setTimeout(() => {
    if (!useStore.getState().urdf) useStore.setState({ urdf: FALLBACK_URDF });
}, 1500);

// ------------------------------------------------------------------ jogging
const cartTimers = new Array(6).fill(null);
const jointTimers = new Array(NJ).fill(null);

/** Velocity jog, refreshed so the controller's dead-man keeps it alive. */
export function cartJogVel(a, v) {
    const line = `cartjogvel ${CART_AXES[a]} ${v.toFixed(4)}`;
    cmd(line);
    clearInterval(cartTimers[a]);
    cartTimers[a] = setInterval(() => cmd(line), JOG_REFRESH_MS);
}
export function stopCartJog(a) {
    if (!cartTimers[a]) return;
    clearInterval(cartTimers[a]);
    cartTimers[a] = null;
    cmd(`cartjogvel ${CART_AXES[a]} 0`);
}
export function cartJogStart(a, dir) {
    const S = useStore.getState().S;
    const v = dir * (a < 3 ? S.cartLinSpeed : S.cartAngSpeed / R2D);
    cartJogVel(a, v);
    useStore.setState({ jogHud: { axis: a, rate: dir } });
}
export function cartStep(a, dir) {
    const S = useStore.getState().S;
    const d = a < 3 ? S.cartLinStep : S.cartAngStep / R2D;
    cmd(`cartjog ${CART_AXES[a]} ${(dir * d).toFixed(5)}`);
}

export function jointJogStart(j, dir) {
    const line = `jogvel ${j + 1} ${(dir * useStore.getState().S.jogSpeed).toFixed(2)}`;
    cmd(line);
    clearInterval(jointTimers[j]);
    jointTimers[j] = setInterval(() => cmd(line), JOG_REFRESH_MS);
}
export function jointJogStop(j) {
    if (!jointTimers[j]) return;
    clearInterval(jointTimers[j]);
    jointTimers[j] = null;
    cmd(`jogvel ${j + 1} 0`);
}
export const jointStep = (j, dir) => cmd(`jog ${j + 1} ${dir * useStore.getState().S.selInc}`);
export const setJoint = (j, v) => cmd(`${j + 1} ${(+v).toFixed(3)}`);

export function stopEverything() {
    for (let a = 0; a < 6; a++) stopCartJog(a);
    for (let j = 0; j < NJ; j++) jointJogStop(j);
    if (useStore.getState().jogHud) useStore.setState({ jogHud: null });
}
addEventListener('blur', stopEverything);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopEverything(); });

export function hardStop() {
    stopEverything();
    send({ type: 'prog_ctl', action: 'stop' });
    cmd('stop');
}

export function setFrame(v) {
    stopEverything();
    setS({ cartFrame: v });
    cmd(`cartframe ${v}`);
}

// ------------------------------------------------------------- misc actions
export function commitPlan(verb) {
    const plan = useStore.getState().plan;
    if (!plan) return;
    const ok = verb === 'movel' ? plan.linOk : plan.jointOk;
    if (!ok) {
        toast.warning(verb === 'movel' ? 'Linear move not possible' : 'Joint move not possible', {
            id: 'blocked', description: verb === 'movel' ? plan.linWhy : 'No joint solution from the current pose',
        });
        return;
    }
    cmd(`${verb} ${[...plan.pos, ...plan.rpy].map((v) => v.toFixed(5)).join(' ')}`);
}

export function saveSettings() {
    send({ type: 'save_settings', values: { ...useStore.getState().S } });
    toast.success('Settings saved');
}
export function resetSettings() {
    useStore.setState({ S: { ...DEFAULTS } });
    send({ type: 'save_settings', values: { ...DEFAULTS } });
    cmd(`cartframe ${DEFAULTS.cartFrame}`);
    toast.success('Settings reset');
}

// Waypoints store JOINT angles: one pose is reachable in up to 8 arm
// configurations, so replaying a pose alone could flip the elbow or wrist.
// `tgt`, not `pos`: the settled planner target carries no encoder noise.
export function teachWp(name) {
    name = (name || '').trim();
    if (!name) { toast.warning('Name the waypoint first'); return null; }
    const { st, tcp } = useStore.getState();
    send({ type: 'save_pose', name, angles: st.tgt.map(Number), pose: tcp.map(Number) });
    toast.success(`Taught “${name}”`);
    return name;
}
export function autoWpName() {
    const poses = useStore.getState().config.poses;
    let i = 1;
    while (poses[`P${i}`]) i++;
    return `P${i}`;
}
export function wpAngles(name) {
    const p = useStore.getState().config.poses[name];
    const a = (Array.isArray(p) ? p : p?.angles) || [];
    return a.length === NJ ? a.map(Number) : null;
}
export function sendGrip(v) {
    cmd(`gripper ${Math.max(0, Math.min(GRIP_MAX, Math.round(v)))}`);
}
