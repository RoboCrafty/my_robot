// Program editor model. A program is a tree of typed steps, not a flat pose
// list: loops need a body, and moves reference waypoints by NAME so re-teaching
// a point updates every program that uses it.
import { toast } from 'sonner';
import { send, useStore, wpAngles } from './store.js';

export const NODE_KINDS = [['move', 'Move'], ['gripper', 'Gripper'], ['wait', 'Wait'],
    ['speed', 'Speed'], ['loop', 'Loop'], ['call', 'Call'], ['comment', 'Note']];

let uidN = 0;
const uid = () => `n${Date.now().toString(36)}${(uidN++).toString(36)}`;

export function newNode(type) {
    const { config, S } = useStore.getState();
    const b = { id: uid(), type };
    switch (type) {
        case 'move': return { ...b, motion: 'j', wp: Object.keys(config.poses)[0] || '', dwell: 0 };
        case 'gripper': return { ...b, value: (config.grips?.[S.grip] || { close: 140 }).close, settle: 0.5 };
        case 'wait': return { ...b, seconds: 1 };
        case 'speed': return { ...b, percent: 50 };
        case 'loop': return { ...b, mode: 'count', count: 3, body: [] };
        case 'call': return { ...b, name: Object.keys(config.programs || {})[0] || '' };
        default: return { ...b, text: '' };
    }
}
const cloneNode = (n) => ({ ...n, id: uid(), ...(n.body ? { body: n.body.map(cloneNode) } : {}) });

export function flatten(nodes, depth = 0, out = []) {
    for (const n of nodes) {
        out.push({ n, depth });
        if (n.type === 'loop') flatten(n.body || [], depth + 1, out);
    }
    return out;
}

// The array a step lives in, its index there, and the loop enclosing it.
export function locate(id, nodes, parent = null) {
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

export function nodeLabel(n) {
    switch (n.type) {
        case 'move': return [`Move ${(n.motion || 'j').toUpperCase()}`,
            (n.wp || '— no waypoint —') + (n.dwell ? ` · dwell ${n.dwell}s` : '')];
        case 'gripper': return ['Grip', `to ${n.value}`];
        case 'wait': return ['Wait', `${n.seconds} s`];
        case 'speed': return ['Speed', `${n.percent}%`];
        case 'loop': return ['Loop', n.mode === 'forever' ? 'forever' : `${n.count} ×`];
        case 'call': return ['Call', n.name || '— no program —'];
        default: return ['Note', n.text || ''];
    }
}

/** Mutate a deep copy of the program; `fn` may return the new selection. */
function edit(fn) {
    useStore.setState((s) => {
        const nodes = structuredClone(s.editor.nodes);
        const sel = fn(nodes, s.editor.sel);
        return { editor: { ...s.editor, nodes, sel: sel === undefined ? s.editor.sel : sel, dirty: true } };
    });
}

export const select = (id) => useStore.setState((s) => ({ editor: { ...s.editor, sel: id } }));

export function insertNode(node) {
    edit((nodes, sel) => {
        const loc = sel && locate(sel, nodes);
        if (!loc) nodes.push(node);
        else if (loc.arr[loc.i].type === 'loop') (loc.arr[loc.i].body ||= []).push(node);
        else loc.arr.splice(loc.i + 1, 0, node);
        return node.id;
    });
}

export function setField(id, k, v) {
    edit((nodes) => { const l = locate(id, nodes); if (l) l.arr[l.i][k] = v; });
}

export const ops = {
    up: () => edit((nodes, sel) => {
        const l = locate(sel, nodes);
        if (l && l.i > 0) [l.arr[l.i - 1], l.arr[l.i]] = [l.arr[l.i], l.arr[l.i - 1]];
    }),
    down: () => edit((nodes, sel) => {
        const l = locate(sel, nodes);
        if (l && l.i < l.arr.length - 1) [l.arr[l.i + 1], l.arr[l.i]] = [l.arr[l.i], l.arr[l.i + 1]];
    }),
    indent: () => {
        const { nodes, sel } = useStore.getState().editor;
        const l = sel && locate(sel, nodes);
        if (!l || l.i === 0 || l.arr[l.i - 1].type !== 'loop') {
            toast.warning('Put a Loop directly above this step first');
            return;
        }
        edit((n) => {
            const m = locate(sel, n);
            (m.arr[m.i - 1].body ||= []).push(m.arr.splice(m.i, 1)[0]);
        });
    },
    outdent: () => edit((nodes, sel) => {
        const l = locate(sel, nodes);
        if (!l || !l.parent) return;
        const p = locate(l.parent.id, nodes);
        p.arr.splice(p.i + 1, 0, l.arr.splice(l.i, 1)[0]);
    }),
    dup: () => edit((nodes, sel) => {
        const l = locate(sel, nodes);
        if (!l) return;
        const copy = cloneNode(l.arr[l.i]);
        l.arr.splice(l.i + 1, 0, copy);
        return copy.id;
    }),
    del: () => edit((nodes, sel) => {
        const l = locate(sel, nodes);
        if (!l) return;
        l.arr.splice(l.i, 1);
        return null;
    }),
};

const confirmDiscard = () => !useStore.getState().editor.dirty
    || confirm('Discard unsaved changes to this program?');

export function loadProgram(name) {
    if (!confirmDiscard()) return;
    const nodes = structuredClone(useStore.getState().config.programs?.[name] || []);
    ensureIds(nodes);
    useStore.setState({ editor: { name, nodes, sel: null, dirty: false } });
}
export function newProgram() {
    if (!confirmDiscard()) return;
    useStore.setState({ editor: { name: '', nodes: [], sel: null, dirty: false } });
}
export function saveProgram() {
    const { editor } = useStore.getState();
    const name = editor.name || (prompt('Program name') || '').trim();
    if (!name) return;
    send({ type: 'save_program', name, nodes: editor.nodes });
    useStore.setState({ editor: { ...editor, name, dirty: false } });
    toast.success(`Saved “${name}”`);
}
export function deleteProgram() {
    const { editor } = useStore.getState();
    if (!editor.name || !confirm(`Delete program “${editor.name}”?`)) return;
    send({ type: 'delete_program', name: editor.name });
    useStore.setState({ editor: { name: '', nodes: [], sel: null, dirty: false } });
}

// Everything the interpreter would otherwise only discover mid-run.
function progIssues() {
    const { editor, config } = useStore.getState();
    const out = [];
    flatten(editor.nodes).forEach(({ n }) => {
        if (n.type === 'move' && !wpAngles(n.wp)) out.push(`Move: waypoint “${n.wp || '—'}” is missing`);
        else if (n.type === 'call' && n.name === editor.name) out.push(`Call: “${n.name}” calls itself`);
        else if (n.type === 'call' && !config.programs?.[n.name]) out.push(`Call: program “${n.name || '—'}” is missing`);
    });
    return out;
}

export function runProgram(step) {
    const { editor, prog } = useStore.getState();
    // Stepping from a standing start also has to *begin* the program.
    if (step && prog.running) { send({ type: 'prog_ctl', action: 'step' }); return; }
    if (!editor.nodes.length) { toast.warning('Program is empty'); return; }
    const bad = progIssues();
    if (bad.length) { toast.error(bad[0]); return; }
    send({ type: 'run', nodes: editor.nodes, step });
}
export const progCtl = (action) => send({ type: 'prog_ctl', action });

/**
 * Joint-angle targets of every move, in execution order (loops once, calls
 * expanded), for the 3D path preview.
 */
export function programMoves(nodes, depth = 0) {
    const { config } = useStore.getState();
    const out = [];
    for (const n of nodes) {
        if (n.type === 'move') {
            const q = wpAngles(n.wp);
            if (q) out.push({ id: n.id, wp: n.wp, motion: n.motion || 'j', q });
        } else if (n.type === 'loop') {
            out.push(...programMoves(n.body || [], depth));
        } else if (n.type === 'call' && depth < 4 && config.programs?.[n.name]) {
            out.push(...programMoves(config.programs[n.name], depth + 1));
        }
    }
    return out;
}
