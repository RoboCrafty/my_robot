import { AnimatePresence, motion } from 'motion/react';
import { R2D, commitPlan, setFrame, setPref, setS, useStore } from '../store.js';
import { targetApi } from '../scene/TargetGizmo.jsx';
import { Icon } from './Icon.jsx';
import { Num, Seg } from './ui.jsx';

const ease = [0.23, 1, 0.32, 1];
export const TCP_VARIANTS = [
    ['a', 'A', 'Ghost + confirm: drag the gizmo, then choose the move'],
    ['b', 'B', 'Ghost + release: the arm moves when you let go'],
    ['c', 'C', 'Velocity: drag a handle and the arm jogs live, farther = faster'],
    ['d', 'D', 'Free drag: pull the bead around the view plane (Shift = up/down)'],
];

export function TargetCard() {
    const variant = useStore((s) => s.prefs.tcp);
    const title = TCP_VARIANTS.find(([v]) => v === variant)?.[2].split(':')[0];

    return (
        <motion.aside
            className="glass target-card"
            initial={{ opacity: 0, x: 24, scale: 0.98 }}
            animate={{ opacity: 1, x: 0, scale: 1 }}
            exit={{ opacity: 0, x: 16, scale: 0.98, transition: { duration: 0.16, ease } }}
        >
            <div className="card-head">
                <div>
                    <div className="eyebrow">Move tool</div>
                    <div className="card-title">{title}</div>
                </div>
                <Seg size="sm" label="TCP control variant" items={TCP_VARIANTS} value={variant}
                    onChange={(v) => setPref('tcp', v)} />
            </div>
            <AnimatePresence mode="popLayout" initial={false}>
                <motion.div key={variant}
                    initial={{ opacity: 0, y: 6 }} animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: -6, transition: { duration: 0.12 } }}
                    transition={{ duration: 0.2, ease }}>
                    {variant === 'c' ? <VelocityBody /> : <GhostBody variant={variant} />}
                </motion.div>
            </AnimatePresence>
        </motion.aside>
    );
}

function GhostBody({ variant }) {
    const plan = useStore((s) => s.plan);
    const preview = useStore((s) => s.preview);
    const auto = useStore((s) => s.prefs.autoMotion);
    const ok = !!plan?.reachable;
    const arrived = plan && plan.dist < 1.5 && plan.ang < 0.5;
    const hoverPath = (p) => ({ onPointerEnter: () => useStore.setState({ preview: p }), onFocus: () => useStore.setState({ preview: p }) });

    return (
        <>
            <div className="delta">
                <div>
                    <span className="delta-v">{plan ? plan.dist.toFixed(1) : '—'}</span>
                    <span className="delta-u">mm away</span>
                </div>
                <div>
                    <span className="delta-v">{plan ? plan.ang.toFixed(1) : '—'}</span>
                    <span className="delta-u">° turn</span>
                </div>
                <span className={`reach ${ok ? 'ok' : 'no'}`}><span className="reach-dot" />{ok ? 'Reachable' : 'Out of reach'}</span>
            </div>

            <dl className="num-grid compact">
                {['X', 'Y', 'Z'].map((k, i) => (
                    <div key={k}><dt>{k}</dt><dd>{plan ? (plan.pos[i] * 1000).toFixed(1) : '—'}</dd></div>
                ))}
                {['RX', 'RY', 'RZ'].map((k, i) => (
                    <div key={k}><dt>{k}</dt><dd>{plan ? (plan.rpy[i] * R2D).toFixed(1) : '—'}</dd></div>
                ))}
            </dl>

            {variant === 'b' ? (
                <div className="auto-row">
                    <span className="muted">Moves on release as</span>
                    <Seg size="sm" label="Motion on release" value={auto}
                        items={[['j', 'Joint'], ['l', 'Linear']]}
                        onChange={(v) => { setPref('autoMotion', v); useStore.setState({ preview: v }); }} />
                </div>
            ) : (
                <div className="go-row">
                    {[['j', 'move', 'curve', 'Joint move', 'Fastest, curved path'],
                      ['l', 'movel', 'line', 'Linear move', 'Straight tool path']].map(([p, verb, icon, label, sub]) => (
                        <motion.button key={p} className={`go ${preview === p ? 'previewing' : ''}`}
                            disabled={!ok || arrived} whileTap={{ scale: 0.97 }}
                            onClick={() => commitPlan(verb)} {...hoverPath(p)}>
                            <Icon name={icon} size={18} />
                            <span><b>{label}</b><small>{sub}</small></span>
                        </motion.button>
                    ))}
                </div>
            )}

            <div className="card-foot">
                <motion.button className="ghost-btn" whileTap={{ scale: 0.96 }} onClick={() => targetApi.reset()}>
                    <Icon name="reset" size={14} /> Snap to tool
                </motion.button>
                <motion.button className="ghost-btn" whileTap={{ scale: 0.96 }} onClick={() => useStore.setState({ mode: 'orbit' })}>
                    <Icon name="close" size={14} /> Done
                </motion.button>
            </div>
        </>
    );
}

function VelocityBody() {
    const S = useStore((s) => s.S);
    return (
        <>
            <p className="muted small">Grab an arrow or ring on the tool and drag along it. The farther you pull, the faster
                it moves; let go to stop.</p>
            <div className="kv">
                <span>Axes</span>
                <Seg size="sm" label="Jog frame" value={S.cartFrame} items={[['base', 'Base'], ['tool', 'Tool']]}
                    onChange={setFrame} />
                <span>Max linear</span>
                <span className="inline"><Num value={S.cartLinSpeed} step={0.01} min={0.001} width={72}
                    onCommit={(v) => setS({ cartLinSpeed: v })} /> m/s</span>
                <span>Max angular</span>
                <span className="inline"><Num value={S.cartAngSpeed} step={1} min={1} width={72}
                    onCommit={(v) => setS({ cartAngSpeed: v })} /> °/s</span>
            </div>
            <div className="card-foot">
                <span />
                <motion.button className="ghost-btn" whileTap={{ scale: 0.96 }} onClick={() => useStore.setState({ mode: 'orbit' })}>
                    <Icon name="close" size={14} /> Done
                </motion.button>
            </div>
        </>
    );
}
