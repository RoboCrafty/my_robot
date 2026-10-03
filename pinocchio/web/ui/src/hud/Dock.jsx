import { motion } from 'motion/react';
import { clearTrail, setS, setView, useStore } from '../store.js';
import { Icon } from './Icon.jsx';

const MODES = [['orbit', 'Orbit', 'orbit'], ['tcp', 'Move tool', 'target']];
const VIEWS = [['iso', 'Iso'], ['front', 'Front'], ['side', 'Side'], ['top', 'Top']];

export function Dock() {
    const mode = useStore((s) => s.mode);
    const view = useStore((s) => s.view);
    const trail = useStore((s) => s.S.trail);

    return (
        <motion.nav
            className="dock"
            initial={{ opacity: 0, y: 16, scale: 0.97 }}
            animate={{ opacity: 1, y: 0, scale: 1 }}
            transition={{ type: 'spring', duration: 0.5, bounce: 0.15, delay: 0.15 }}
        >
            <div className="dock-seg" role="tablist" aria-label="Interaction mode">
                {MODES.map(([v, label, icon]) => (
                    <motion.button
                        key={v}
                        role="tab"
                        aria-selected={mode === v}
                        className={`dock-btn ${mode === v ? 'on' : ''}`}
                        whileTap={{ scale: 0.96 }}
                        onClick={() => useStore.setState({ mode: v })}
                    >
                        {mode === v && <motion.span layoutId="dock-pill" className="dock-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.18 }} />}
                        <Icon name={icon} />
                        <span>{label}</span>
                        {v === 'tcp' && <kbd>T</kbd>}
                    </motion.button>
                ))}
            </div>

            <span className="dock-sep" aria-hidden />

            <div className="views" role="group" aria-label="Camera view">
                {VIEWS.map(([v, label]) => (
                    <motion.button key={v} className={`view-btn ${view === v ? 'on' : ''}`} whileTap={{ scale: 0.94 }} onClick={() => setView(v)}>
                        {label}
                        {view === v && <motion.span layoutId="view-dot" className="view-dot" />}
                    </motion.button>
                ))}
            </div>

            <span className="dock-sep" aria-hidden />

            <motion.button
                className={`icon-btn ${trail ? 'on' : ''}`}
                whileTap={{ scale: 0.92 }}
                aria-pressed={trail}
                title={trail ? 'Hide TCP trail' : 'Show TCP trail'}
                onClick={() => setS({ trail: !trail })}
            >
                <Icon name="trail" />
            </motion.button>
            <motion.button className="icon-btn" whileTap={{ scale: 0.92 }} title="Clear trail" onClick={clearTrail}>
                <Icon name="erase" />
            </motion.button>
        </motion.nav>
    );
}
