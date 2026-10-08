import { motion } from 'motion/react';
import { R2D, useStore } from '../store.js';

export function Readout() {
    const t = useStore((s) => s.tcp);
    const sigma = useStore((s) => s.sigma);
    const level = sigma < 0.02 ? 'bad' : sigma < 0.05 ? 'warn' : 'good';

    return (
        <motion.section
            className="glass readout"
            initial={{ opacity: 0, y: -8 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ type: 'spring', duration: 0.5, bounce: 0.1, delay: 0.1 }}
        >
            <div className="eyebrow">Tool centre point</div>
            <dl className="num-grid">
                {['X', 'Y', 'Z'].map((k, i) => (
                    <div key={k}><dt>{k}</dt><dd>{(t[i] * 1000).toFixed(1)}<small>mm</small></dd></div>
                ))}
                {['RX', 'RY', 'RZ'].map((k, i) => (
                    <div key={k}><dt>{k}</dt><dd>{(t[i + 3] * R2D).toFixed(1)}<small>°</small></dd></div>
                ))}
            </dl>
            <div className={`sigma ${level}`} title="Smallest singular value of the Jacobian">
                <span>Manipulability</span>
                <span className="sigma-bar"><motion.i animate={{ scaleX: Math.min(1, sigma / 0.15) }} /></span>
                <b>{sigma.toFixed(3)}</b>
            </div>
        </motion.section>
    );
}
