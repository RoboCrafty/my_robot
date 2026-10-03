import { useRef, useState } from 'react';
import { motion } from 'motion/react';
import { NJ, cmd, jointJogStart, jointJogStop, jointStep, setJoint, setS, useStore } from '../store.js';
import { HoldButton, Num, Section, Seg } from '../hud/ui.jsx';
import { Icon } from '../hud/Icon.jsx';

export function JointsPane() {
    const S = useStore((s) => s.S);
    const increments = useStore((s) => s.config.increments) || [];
    const enabled = useStore((s) => s.st.enabled);
    return (
        <>
            <Section>
                <div className="tool-row">
                    <span className="muted small">Step</span>
                    <Seg size="sm" label="Joint step" value={S.selInc} onChange={(v) => setS({ selInc: v })}
                        items={increments.map((v) => [v, `${v}°`])} />
                    <span className="spacer" />
                    <Num value={S.jogSpeed} step={1} min={1} width={60} onCommit={(v) => setS({ jogSpeed: v })} />
                    <span className="muted small">°/s</span>
                </div>
            </Section>
            <Section>
                <div className="joints">
                    {Array.from({ length: NJ }, (_, j) => <JointRow key={j} j={j} />)}
                </div>
            </Section>
            <Section title="Motor torque">
                <div className="motors">
                    {enabled.map((on, j) => (
                        <motion.button key={j} whileTap={{ scale: 0.94 }} className={`motor ${on ? 'on' : ''}`}
                            title={on ? 'torque on — click to disable' : 'torque off — click to enable'}
                            onClick={() => cmd(`motor ${j + 1} ${on ? 'off' : 'on'}`)}>
                            <span className="motor-led" />J{j + 1}
                        </motion.button>
                    ))}
                </div>
            </Section>
        </>
    );
}

function JointRow({ j }) {
    const lim = useStore((s) => s.limits?.[j]) || { lower: -180, upper: 180 };
    const pos = useStore((s) => s.st.pos[j]);
    const tgt = useStore((s) => s.st.tgt[j]);
    const on = !!useStore((s) => s.st.enabled[j]);
    const hold = useStore((s) => s.S.jogMode) === 'hold';
    // While the user drags, show their value; the controller's target catches up.
    const [local, setLocal] = useState(null);
    const lastSend = useRef(0);
    const release = useRef(null);
    const value = local ?? +tgt;
    const span = lim.upper - lim.lower;
    const pct = (v) => `${((v - lim.lower) / span) * 100}%`;

    const onInput = (v) => {
        setLocal(v);
        clearTimeout(release.current);
        const now = performance.now();
        if (now - lastSend.current > 40) { lastSend.current = now; setJoint(j, v); }
    };
    const onDone = () => {
        if (local != null) setJoint(j, local);
        release.current = setTimeout(() => setLocal(null), 700);
    };

    return (
        <div className={`jrow ${on ? '' : 'off'}`}>
            <div className="jrow-head">
                <span className="jname">J{j + 1}</span>
                <span className="jact mono">{(+pos).toFixed(1)}°</span>
                <span className="spacer" />
                <Num value={(+tgt).toFixed(1)} step={0.1} width={68} disabled={!on} onCommit={(v) => setJoint(j, v)} />
                <motion.button whileTap={{ scale: 0.92 }} className="icon-btn sm" disabled={!on} title="joint to 0" onClick={() => cmd(`${j + 1} 0`)}>0</motion.button>
            </div>
            <div className="jrow-ctl">
                <HoldButton className="jstep" hold={hold} disabled={!on} aria-label={`J${j + 1} minus`}
                    onStart={() => jointJogStart(j, -1)} onStop={() => jointJogStop(j)} onTap={() => jointStep(j, -1)}>
                    <Icon name="minus" size={14} />
                </HoldButton>
                <div className="jslider">
                    <input type="range" min={lim.lower} max={lim.upper} step={0.1} value={value} disabled={!on}
                        style={{ '--fill': pct(value) }}
                        onChange={(e) => onInput(+e.target.value)}
                        onPointerUp={onDone} onKeyUp={onDone} onBlur={onDone} />
                    {/* Actual position, so following error is visible at a glance. */}
                    <span className="jact-mark" style={{ left: pct(+pos) }} />
                </div>
                <HoldButton className="jstep" hold={hold} disabled={!on} aria-label={`J${j + 1} plus`}
                    onStart={() => jointJogStart(j, 1)} onStop={() => jointJogStop(j)} onTap={() => jointStep(j, 1)}>
                    <Icon name="plus" size={14} />
                </HoldButton>
            </div>
        </div>
    );
}
