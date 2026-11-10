import { useEffect, useRef, useState } from 'react';
import { motion } from 'motion/react';
import { toast } from 'sonner';
import { GRIP_MAX, autoWpName, cmd, teachWp, useStore, wpAngles } from '../store.js';
import {
    NODE_KINDS, deleteProgram, flatten, insertNode, loadProgram, locate, newNode, newProgram, nodeLabel,
    ops, progCtl, runProgram, saveProgram, select, setField,
} from '../program.js';
import { Badge, Button, Field, Num, Section, Seg } from '../hud/ui.jsx';
import { Icon } from '../hud/Icon.jsx';

export function ProgramPane() {
    const programs = useStore((s) => s.config.programs) || {};
    const editor = useStore((s) => s.editor);
    const prog = useStore((s) => s.prog);
    const names = Object.keys(programs);

    const status = prog.error ? ['bad', prog.error]
        : !prog.running ? ['', 'idle']
        : prog.paused ? ['warn', 'paused'] : ['good', 'running'];

    return (
        <>
            <Section>
                <div className="tool-row">
                    <select className="text grow" value={editor.name} onChange={(e) => loadProgram(e.target.value)} title="saved programs">
                        {!editor.name && <option value="">— untitled —</option>}
                        {names.map((n) => <option key={n} value={n}>{n}</option>)}
                    </select>
                    <Button size="sm" onClick={newProgram} title="start an empty program">New</Button>
                    <Button size="sm" variant={editor.dirty ? 'accent' : 'default'} onClick={saveProgram}>Save</Button>
                    <Button size="sm" variant="ghost danger" onClick={deleteProgram} title="delete the saved program"><Icon name="trash" size={14} /></Button>
                </div>
                <div className="transport">
                    <Button variant="accent" onClick={() => runProgram(false)} title="run from the top"><Icon name="play" size={14} /> Run</Button>
                    <Button onClick={() => progCtl(prog.paused ? 'resume' : 'pause')} disabled={!prog.running}
                        title={prog.paused ? 'resume' : 'pause'}><Icon name={prog.paused ? 'play' : 'pause'} size={14} /></Button>
                    <Button onClick={() => runProgram(true)} title="execute one step"><Icon name="step" size={14} /></Button>
                    <Button variant="danger" onClick={() => progCtl('stop')} title="stop"><Icon name="square" size={14} /></Button>
                    <span className="spacer" />
                    <Badge tone={status[0]}>{status[1]}</Badge>
                </div>
                <SpeedOverride />
            </Section>

            <Section title="Insert">
                <div className="palette">
                    <Button size="sm" variant="accent-soft" title="Teach the current position as a new waypoint, then move to it"
                        onClick={() => { const n = teachWp(autoWpName()); if (n) insertNode({ ...newNode('move'), wp: n }); }}>
                        <Icon name="plus" size={13} /> Here
                    </Button>
                    {NODE_KINDS.map(([type, label]) => (
                        <Button key={type} size="sm" onClick={() => insertNode(newNode(type))} title={`insert a ${label} step`}>{label}</Button>
                    ))}
                </div>
            </Section>

            <Section title="Steps" aside={<span className="muted small">path shown in 3D</span>}>
                <Tree />
                <div className="tool-row">
                    <Button size="xs" variant="ghost" onClick={ops.up} title="move up"><Icon name="up" size={14} /></Button>
                    <Button size="xs" variant="ghost" onClick={ops.down} title="move down"><Icon name="down" size={14} /></Button>
                    <Button size="xs" variant="ghost" onClick={ops.indent} title="move into the loop above"><Icon name="indent" size={14} /></Button>
                    <Button size="xs" variant="ghost" onClick={ops.outdent} title="move out of the loop"><Icon name="outdent" size={14} /></Button>
                    <Button size="xs" variant="ghost" onClick={ops.dup} title="duplicate"><Icon name="copy" size={14} /></Button>
                    <span className="spacer" />
                    <Button size="xs" variant="ghost danger" onClick={ops.del} title="delete step"><Icon name="trash" size={14} /></Button>
                </div>
            </Section>

            <Inspector />
        </>
    );
}

function SpeedOverride() {
    const speed = useStore((s) => s.st.speed);
    const [local, setLocal] = useState(null);
    const t = useRef(null);
    const v = local ?? speed;
    return (
        <div className="slider-row" title="global feed override — dry-run a new program slowly">
            <span className="muted small">Speed</span>
            <input type="range" min={5} max={100} step={5} value={v} style={{ '--fill': `${((v - 5) / 95) * 100}%` }}
                onChange={(e) => {
                    setLocal(+e.target.value);
                    cmd(`speed ${e.target.value}`);
                    clearTimeout(t.current);
                    t.current = setTimeout(() => setLocal(null), 700);
                }} />
            <span className="mono val">{Math.round(v)}%</span>
        </div>
    );
}

function Tree() {
    const nodes = useStore((s) => s.editor.nodes);
    const sel = useStore((s) => s.editor.sel);
    const runNode = useStore((s) => s.prog.node);
    const counters = useStore((s) => s.prog.counters);
    const ref = useRef();

    useEffect(() => {
        if (runNode) ref.current?.querySelector('.node.run')?.scrollIntoView({ block: 'nearest' });
    }, [runNode]);

    const rows = flatten(nodes);
    if (!rows.length) return <div className="empty">Empty program. Teach a point with “+ Here”, or insert steps above.</div>;
    return (
        <div className="tree" ref={ref}>
            {rows.map(({ n, depth }, i) => {
                const [kind, txt] = nodeLabel(n);
                return (
                    <motion.div key={n.id} layout="position" transition={{ duration: 0.18, ease: [0.23, 1, 0.32, 1] }}
                        className={`node ${n.id === sel ? 'sel' : ''} ${n.id === runNode ? 'run' : ''} ${n.type}`}
                        onPointerEnter={() => n.type === 'move' && useStore.setState({ hoverWp: n.wp })}
                        onPointerLeave={() => useStore.setState({ hoverWp: null })}
                        onClick={() => select(n.id)}>
                        <span className="node-n mono">{i + 1}</span>
                        {Array.from({ length: depth }, (_, k) => <span key={k} className="rail" />)}
                        <span className={`kind k-${n.type}${n.type === 'move' ? `-${n.motion || 'j'}` : ''}`}>{kind}</span>
                        <span className="node-txt">{txt}</span>
                        {counters[n.id] && <Badge tone="good">{counters[n.id]}</Badge>}
                    </motion.div>
                );
            })}
        </div>
    );
}

function Inspector() {
    const nodes = useStore((s) => s.editor.nodes);
    const sel = useStore((s) => s.editor.sel);
    const progName = useStore((s) => s.editor.name);
    const poses = useStore((s) => s.config.poses);
    const programs = useStore((s) => s.config.programs) || {};
    const grips = useStore((s) => s.config.grips) || {};
    const gripSel = useStore((s) => s.S.grip);
    const loc = sel && locate(sel, nodes);
    if (!loc) return null;
    const n = loc.arr[loc.i];
    const set = (k, v) => setField(n.id, k, v);
    const g = grips[gripSel] || { open: 0, close: GRIP_MAX };

    return (
        <Section title="Step details">
            <div className="inspector">
                {n.type === 'move' && (
                    <>
                        <Field label="Motion">
                            <Seg grow size="sm" label="Motion" value={n.motion || 'j'} onChange={(v) => set('motion', v)}
                                items={[['j', 'Joint', 'fastest, path is not straight'], ['l', 'Linear', 'straight line in space']]} />
                        </Field>
                        <Field label="Waypoint">
                            <select className="text grow" value={n.wp} onChange={(e) => set('wp', e.target.value)}>
                                {Object.keys(poses).length === 0 && <option value="">—</option>}
                                {Object.keys(poses).map((p) => <option key={p} value={p}>{p}</option>)}
                            </select>
                        </Field>
                        <Field label="Dwell s"><Num value={n.dwell || 0} step={0.1} min={0} onCommit={(v) => set('dwell', Math.max(0, v))} /></Field>
                        <Button size="sm" onClick={() => {
                            const a = wpAngles(n.wp);
                            if (!a) { toast.error(`Waypoint “${n.wp}” is missing`); return; }
                            const j = a.map((x) => x.toFixed(3)).join(' ');
                            cmd(n.motion === 'l' ? `movel q ${j}` : j);
                        }}>Preview this move</Button>
                    </>
                )}
                {n.type === 'gripper' && (
                    <>
                        <Field label="Position">
                            <span className="slider-row">
                                <input type="range" min={0} max={GRIP_MAX} step={1} defaultValue={n.value} key={n.id + n.value}
                                    style={{ '--fill': `${(n.value / GRIP_MAX) * 100}%` }}
                                    onPointerUp={(e) => set('value', +e.target.value)} onKeyUp={(e) => set('value', +e.target.value)} />
                                <span className="mono val">{n.value}</span>
                            </span>
                        </Field>
                        <Field label="From preset">
                            <span className="row2">
                                <Button size="sm" onClick={() => set('value', g.open)}>Open</Button>
                                <Button size="sm" onClick={() => set('value', g.close)}>Close</Button>
                            </span>
                        </Field>
                        <Field label="Settle s"><Num value={n.settle ?? 0.5} step={0.1} min={0} onCommit={(v) => set('settle', Math.max(0, v))} /></Field>
                    </>
                )}
                {n.type === 'wait' && <Field label="Seconds"><Num value={n.seconds} step={0.1} min={0} onCommit={(v) => set('seconds', Math.max(0, v))} /></Field>}
                {n.type === 'speed' && (
                    <Field label="Percent"><Num value={n.percent} step={5} min={5} max={100}
                        onCommit={(v) => set('percent', Math.max(5, Math.min(100, Math.round(v))))} /></Field>
                )}
                {n.type === 'loop' && (
                    <>
                        <Field label="Mode">
                            <Seg grow size="sm" label="Loop mode" value={n.mode} onChange={(v) => set('mode', v)}
                                items={[['count', 'Count'], ['forever', 'Forever']]} />
                        </Field>
                        {n.mode !== 'forever' && (
                            <Field label="Repeats"><Num value={n.count} step={1} min={1} onCommit={(v) => set('count', Math.max(1, Math.round(v)))} /></Field>
                        )}
                    </>
                )}
                {n.type === 'call' && (
                    <Field label="Program">
                        <select className="text grow" value={n.name} onChange={(e) => set('name', e.target.value)}>
                            <option value="">—</option>
                            {Object.keys(programs).filter((x) => x !== progName).map((p) => <option key={p} value={p}>{p}</option>)}
                        </select>
                    </Field>
                )}
                {n.type === 'comment' && (
                    <Field label="Text">
                        <input className="text grow" type="text" defaultValue={n.text || ''} key={n.id} placeholder="note"
                            onBlur={(e) => set('text', e.target.value)} />
                    </Field>
                )}
            </div>
        </Section>
    );
}
