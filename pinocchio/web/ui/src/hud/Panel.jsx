import { AnimatePresence, motion } from 'motion/react';
import { setS, useStore } from '../store.js';
import { Icon } from './Icon.jsx';
import { JogPane } from '../panes/JogPane.jsx';
import { JointsPane } from '../panes/JointsPane.jsx';
import { PointsPane } from '../panes/PointsPane.jsx';
import { ProgramPane } from '../panes/ProgramPane.jsx';
import { SetupPane } from '../panes/SetupPane.jsx';

const TABS = [
    ['jog', 'Jog', 'bolt', JogPane],
    ['joints', 'Joints', 'joints', JointsPane],
    ['points', 'Points', 'pin', PointsPane],
    ['program', 'Program', 'list', ProgramPane],
    ['setup', 'Setup', 'gear', SetupPane],
];

export function Panel() {
    const open = useStore((s) => s.prefs.panel);
    const tab = useStore((s) => s.S.tab);
    const running = useStore((s) => s.prog.running);
    const current = TABS.find(([v]) => v === tab) || TABS[0];
    const Pane = current[3];

    return (
        <AnimatePresence>
            {open && (
                <motion.aside
                    className="glass panel"
                    initial={{ opacity: 0, x: 40 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: 40, transition: { duration: 0.2, ease: [0.23, 1, 0.32, 1] } }}
                    transition={{ type: 'spring', duration: 0.45, bounce: 0.1 }}
                >
                    <nav className="tabs" role="tablist">
                        {TABS.map(([v, label, icon]) => (
                            <motion.button key={v} role="tab" aria-selected={tab === v}
                                className={`tab ${tab === v ? 'on' : ''}`} whileTap={{ scale: 0.95 }}
                                onClick={() => setS({ tab: v })}>
                                {tab === v && <motion.span layoutId="tab-pill" className="tab-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.15 }} />}
                                <Icon name={icon} size={15} />
                                <span>{label}</span>
                                {v === 'program' && running && <span className="tab-live" aria-label="running" />}
                            </motion.button>
                        ))}
                    </nav>
                    <div className="pane-scroll">
                        {/* Tabs switch often: swap instantly, no animation. */}
                        <div className="pane" key={current[0]}><Pane /></div>
                    </div>
                </motion.aside>
            )}
        </AnimatePresence>
    );
}
