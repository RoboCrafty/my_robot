import { useRef, useState } from 'react';
import {
    CART_AXES, GRIP_MAX, R2D, cartJogStart, cartStep, send, sendGrip, setFrame, setS, stopCartJog, useStore,
} from '../store.js';
import { AXIS_COLORS } from '../scene/urdf.js';
import { Badge, Button, HoldButton, Num, Section, Seg } from '../hud/ui.jsx';
import { toast } from 'sonner';

export function JogPane() {
    const S = useStore((s) => s.S);
    const axesNote = S.cartFrame === 'tool' ? 'tool axes' : 'base axes';
    return (
        <>
            <Section>
                <div className="row2">
                    <Seg grow label="Jog mode" value={S.jogMode} onChange={(v) => setS({ jogMode: v })}
                        items={[['hold', 'Hold', 'move while pressed'], ['step', 'Step', 'one increment per tap']]} />
                    <Seg grow label="Frame" value={S.cartFrame} onChange={setFrame}
                        items={[['base', 'Base'], ['tool', 'Tool']]} />
                </div>
            </Section>

            <Section title="Translate" aside={<span className="muted small">{axesNote}</span>}>
                <JogPad axes={[0, 1, 2]} />
                <div className="tool-row">
                    <span className="muted small">Step</span>
                    <Seg size="sm" label="Linear step" value={S.cartLinStep} onChange={(v) => setS({ cartLinStep: v })}
                        items={[0.001, 0.005, 0.01, 0.05].map((v) => [v, `${v * 1000}`])} />
                    <span className="muted small">mm</span>
                    <span className="spacer" />
                    <Num value={S.cartLinSpeed} step={0.01} min={0.001} width={64} onCommit={(v) => setS({ cartLinSpeed: v })} />
                    <span className="muted small">m/s</span>
                </div>
            </Section>

            <Section title="Rotate about TCP" aside={<span className="muted small">{axesNote}</span>}>
                <JogPad axes={[3, 4, 5]} />
                <div className="tool-row">
                    <span className="muted small">Step</span>
                    <Seg size="sm" label="Angular step" value={S.cartAngStep} onChange={(v) => setS({ cartAngStep: v })}
                        items={[1, 5, 15, 45].map((v) => [v, `${v}`])} />
                    <span className="muted small">°</span>
                    <span className="spacer" />
                    <Num value={S.cartAngSpeed} step={1} min={1} width={64} onCommit={(v) => setS({ cartAngSpeed: v })} />
                    <span className="muted small">°/s</span>
                </div>
            </Section>

            <Gripper />
        </>
    );
}

function JogPad({ axes }) {
    const hold = useStore((s) => s.S.jogMode) === 'hold';
    const tcp = useStore((s) => s.tcp);
    return (
        <div className="jogpad">
            {axes.map((a) => {
                const rot = a >= 3;
                const val = rot ? `${(tcp[a] * R2D).toFixed(1)}°` : `${(tcp[a] * 1000).toFixed(1)}`;
                const common = {
                    hold,
                    onStop: () => { stopCartJog(a); useStore.setState({ jogHud: null }); },
                    onEnter: () => useStore.setState({ hoverAxis: a }),
                    onLeave: () => useStore.setState({ hoverAxis: null }),
                };
                return (
                    <div key={a} className="jogrow" style={{ '--ax': AXIS_COLORS[a % 3] }}>
                        <HoldButton className="jogbtn" aria-label={`${CART_AXES[a]} minus`} {...common}
                            onStart={() => cartJogStart(a, -1)} onTap={() => cartStep(a, -1)}>
                            <Arrow dir={-1} rot={rot} />
                        </HoldButton>
                        <div className="jog-mid">
                            <span className="jog-ax">{CART_AXES[a].toUpperCase()}</span>
                            <span className="jog-val">{val}{!rot && <small>mm</small>}</span>
                        </div>
                        <HoldButton className="jogbtn" aria-label={`${CART_AXES[a]} plus`} {...common}
                            onStart={() => cartJogStart(a, 1)} onTap={() => cartStep(a, 1)}>
                            <Arrow dir={1} rot={rot} />
                        </HoldButton>
                    </div>
                );
            })}
        </div>
    );
}

function Arrow({ dir, rot }) {
    if (!rot) {
        return (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
                <path d={dir > 0 ? 'M5 12h13M13 7l5 5-5 5' : 'M19 12H6M11 7l-5 5 5 5'} />
            </svg>
        );
    }
    return (
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
            <g transform={dir > 0 ? undefined : 'scale(-1,1) translate(-24,0)'}>
                <path d="M5 14a7 7 0 0 1 14 0" /><path d="M19 14l-3-3M19 14l3-3" />
            </g>
        </svg>
    );
}

// The controller only knows raw 0..140. Named presets ("how far to close for
// THIS part") are task metadata and live in poses.json with the poses.
function Gripper() {
    const grips = useStore((s) => s.config.grips) || {};
    const sel = useStore((s) => s.S.grip);
    const st = useStore((s) => s.st);
    const [local, setLocal] = useState(null);
    const busy = useRef(0);
    const [name, setName] = useState('');
    const names = Object.keys(grips);
    const g = grips[sel] || { open: 0, close: GRIP_MAX };
    const value = local ?? st.grip;

    const go = (v) => { setLocal(v); busy.current = Date.now(); sendGrip(v); setTimeout(() => { if (Date.now() - busy.current >= 700) setLocal(null); }, 750); };

    // The ESP echoes what ESP-NOW actually delivered; a lasting mismatch means
    // the gripper ESP is unreachable, not just mid-travel.
    const ack = st.gripAck;
    const ackBadge = typeof ack !== 'number' ? null
        : ack === st.grip ? <Badge tone="good">at {ack}</Badge> : <Badge tone="warn">sending {st.grip}→</Badge>;

    return (
        <Section title="Gripper" aside={ackBadge}>
            <div className="row2">
                <Button onClick={() => go(g.open)} title={`open to ${g.open}`}>Open</Button>
                <Button variant="accent" onClick={() => go(g.close)} title={`close to ${g.close}`}>Close</Button>
            </div>
            <div className="slider-row">
                <input type="range" min={0} max={GRIP_MAX} step={1} value={value}
                    style={{ '--fill': `${(value / GRIP_MAX) * 100}%` }}
                    onChange={(e) => go(+e.target.value)} />
                <span className="mono val">{value}</span>
            </div>
            {names.length > 0 && (
                <Seg grow size="sm" label="Grip preset" value={sel} onChange={(v) => setS({ grip: v })}
                    items={names.map((n) => [n, n])} />
            )}
            <div className="tool-row">
                <input type="text" className="text grow" placeholder="object name" value={name} onChange={(e) => setName(e.target.value)} />
                <Button size="sm" title="Save the current slider value as this preset's closed position" onClick={() => {
                    const n = name.trim() || sel;
                    if (!n) return;
                    send({ type: 'save_grip', name: n, open: g.open, close: value });
                    setS({ grip: n });
                    setName('');
                    toast.success(`Grip “${n}” closes at ${value}`);
                }}>Teach close</Button>
            </div>
        </Section>
    );
}
