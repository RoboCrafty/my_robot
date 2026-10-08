import { motion } from 'motion/react';
import { cmd, setPref, stopEverything, useStore, hardStop } from '../store.js';
import { Icon } from './Icon.jsx';

export function TopBar() {
    const conn = useStore((s) => s.conn);
    const sim = useStore((s) => s.st.sim);
    const canSimOff = useStore((s) => s.st.canSimOff);
    const theme = useStore((s) => s.prefs.theme);
    const panel = useStore((s) => s.prefs.panel);

    const toggleSim = () => {
        if (sim && !confirm('Leave the simulator and drive the real arm?')) return;
        stopEverything();
        cmd(`sim ${sim ? 'off' : 'on'}`);
    };

    return (
        <header className="topbar">
            <div className="brand">
                <span className="mark" aria-hidden />
                <div>
                    <div className="brand-name">Parol6</div>
                    <div className="brand-sub">
                        <span className={`dot ${conn}`} />
                        {conn === 'ok' ? 'Controller linked' : conn === 'down' ? 'Reconnecting' : 'Connecting'}
                    </div>
                </div>
            </div>

            <div className="top-actions">
                <div className="glass pill-group">
                    <motion.button whileTap={{ scale: 0.95 }} className="pill" title="Align the planner to motor feedback (no motion)" onClick={() => cmd('sync')}>
                        <Icon name="sync" size={14} /> Sync
                    </motion.button>
                    <motion.button whileTap={{ scale: 0.95 }} className="pill" onClick={() => cmd('home')}>
                        <Icon name="home" size={14} /> Home
                    </motion.button>
                    <motion.button whileTap={{ scale: 0.95 }} className="pill" onClick={() => cmd('ready')}>
                        <Icon name="ready" size={14} /> Ready
                    </motion.button>
                    <motion.button whileTap={{ scale: 0.95 }} className="pill danger-text"
                        onClick={() => { if (confirm('Rehome all joints using the limit switches?')) cmd('rehome'); }}>
                        Rehome
                    </motion.button>
                </div>

                {sim !== null && (
                    <motion.button
                        whileTap={{ scale: 0.95 }}
                        className={`glass mode-chip ${sim ? 'sim' : 'live'}`}
                        disabled={sim && !canSimOff}
                        title={sim && !canSimOff
                            ? 'No serial port open — restart with a real port to leave the simulator'
                            : sim ? 'Simulated — click to switch to hardware' : 'Driving real hardware — click to switch to the simulator'}
                        onClick={toggleSim}
                    >
                        <span className="dot" /> {sim ? 'SIM' : 'LIVE'}
                    </motion.button>
                )}

                <motion.button whileTap={{ scale: 0.92 }} className="glass round" title={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
                    onClick={() => setPref('theme', theme === 'dark' ? 'light' : 'dark')}>
                    <Icon name={theme === 'dark' ? 'sun' : 'moon'} />
                </motion.button>
                <motion.button whileTap={{ scale: 0.92 }} className={`glass round ${panel ? 'on' : ''}`} title={panel ? 'Hide panel' : 'Show panel'}
                    onClick={() => setPref('panel', !panel)}>
                    <Icon name="panel" />
                </motion.button>

                <motion.button className="estop" whileTap={{ scale: 0.95 }} onClick={hardStop} title="Stop everything (Space)">
                    <span className="estop-ring" aria-hidden />
                    Stop
                </motion.button>
            </div>
        </header>
    );
}
