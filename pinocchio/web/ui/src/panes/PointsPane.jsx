import { useState } from 'react';
import { AnimatePresence, motion } from 'motion/react';
import { cmd, send, teachWp, useStore } from '../store.js';
import { Button, Section } from '../hud/ui.jsx';
import { Icon } from '../hud/Icon.jsx';

export function PointsPane() {
    const poses = useStore((s) => s.config.poses);
    const [name, setName] = useState('');
    const entries = Object.entries(poses).filter(([, p]) => ((Array.isArray(p) ? p : p?.angles) || []).length === 6);

    const teach = () => { if (teachWp(name)) setName(''); };

    return (
        <>
            <Section>
                <div className="tool-row">
                    <input className="text grow" type="text" placeholder="waypoint name" value={name}
                        onChange={(e) => setName(e.target.value)} onKeyDown={(e) => { if (e.key === 'Enter') teach(); }} />
                    <Button variant="accent" size="sm" onClick={teach}><Icon name="pin" size={14} /> Teach here</Button>
                </div>
                <p className="muted small">A waypoint stores <b>joint angles</b>: a TCP pose alone does not say which arm
                    configuration reaches it. The pose shown is the controller's own FK, for reference.</p>
            </Section>
            <Section title={`${entries.length} waypoint${entries.length === 1 ? '' : 's'}`}>
                {entries.length === 0 && <div className="empty">Jog the arm somewhere useful, name it, and teach it.</div>}
                <motion.ul className="list" layout>
                    <AnimatePresence initial={false}>
                        {entries.map(([n, p]) => <WpItem key={n} name={n} p={p} />)}
                    </AnimatePresence>
                </motion.ul>
            </Section>
        </>
    );
}

function WpItem({ name, p }) {
    const a = (Array.isArray(p) ? p : p.angles).map(Number);
    const tcp = Array.isArray(p) ? null : p.pose;
    const j = a.map((x) => x.toFixed(3)).join(' ');
    return (
        <motion.li className="item" layout
            initial={{ opacity: 0, y: -4 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, height: 0, transition: { duration: 0.15 } }}
            onPointerEnter={() => useStore.setState({ hoverWp: name })}
            onPointerLeave={() => useStore.setState({ hoverWp: null })}>
            <div className="item-main">
                <span className="item-name">{name}</span>
                <span className="item-sub mono">
                    {tcp ? tcp.slice(0, 3).map((v) => (v * 1000).toFixed(0)).join('  ') + ' mm' : a.map((x) => x.toFixed(0)).join(', ')}
                </span>
            </div>
            <div className="item-actions">
                <Button size="xs" title="joint move to here" onClick={() => cmd(j)}>J</Button>
                <Button size="xs" title="straight-line move to here" onClick={() => cmd(`movel q ${j}`)}>L</Button>
                <Button size="xs" variant="ghost" title="re-teach at the current position" onClick={() => teachWp(name)}><Icon name="reset" size={13} /></Button>
                <Button size="xs" variant="ghost danger" title="delete" onClick={() => {
                    if (confirm(`Delete waypoint “${name}”? Programs using it will refuse to run.`)) send({ type: 'delete_pose', name });
                }}><Icon name="trash" size={13} /></Button>
            </div>
        </motion.li>
    );
}
