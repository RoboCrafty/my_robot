import { AnimatePresence, motion } from 'motion/react';
import { CART_AXES, useStore } from '../store.js';
import { AXIS_COLORS } from '../scene/urdf.js';

// Live speed feedback while a velocity jog is running, so a short drag visibly
// reads as a slow move even while your eyes are on the arm.
export function JogHud() {
    const hud = useStore((s) => s.jogHud);
    const S = useStore((s) => s.S);
    return (
        <AnimatePresence>
            {hud && (
                <motion.div
                    className="glass jog-hud"
                    style={{ '--ax': AXIS_COLORS[hud.axis % 3] }}
                    initial={{ opacity: 0, y: 8, scale: 0.97 }}
                    animate={{ opacity: 1, y: 0, scale: 1 }}
                    exit={{ opacity: 0, y: 4, transition: { duration: 0.12 } }}
                >
                    <b>{CART_AXES[hud.axis].toUpperCase()}</b>
                    <span className="hud-bar">
                        <i style={{
                            left: hud.rate >= 0 ? '50%' : `${50 - Math.min(1, -hud.rate) * 50}%`,
                            width: `${Math.min(1, Math.abs(hud.rate)) * 50}%`,
                        }} />
                    </span>
                    <span className="hud-val">
                        {hud.axis < 3
                            ? `${(hud.rate * S.cartLinSpeed * 1000).toFixed(1)} mm/s`
                            : `${(hud.rate * S.cartAngSpeed).toFixed(1)} °/s`}
                    </span>
                </motion.div>
            )}
        </AnimatePresence>
    );
}
