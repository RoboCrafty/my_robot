import { useEffect, useId, useRef, useState } from 'react';
import { motion } from 'motion/react';

const press = { scale: 0.97 };

export function Button({ variant = 'default', size = 'md', className = '', children, ...rest }) {
    return (
        <motion.button whileTap={rest.disabled ? undefined : press}
            className={`btn ${variant} ${size} ${className}`} {...rest}>
            {children}
        </motion.button>
    );
}

/** Segmented control with a pill that slides to the selected option. */
export function Seg({ items, value, onChange, grow, size = 'md', label }) {
    const id = useId();
    return (
        <div className={`seg ${grow ? 'grow' : ''} ${size}`} role="radiogroup" aria-label={label}>
            {items.map(([v, text, title]) => {
                const on = v === value;
                return (
                    <motion.button key={String(v)} role="radio" aria-checked={on} title={title}
                        className={`seg-opt ${on ? 'on' : ''}`} whileTap={press} onClick={() => onChange(v)}>
                        {on && <motion.span layoutId={`pill-${id}`} className="seg-pill" transition={{ type: 'spring', duration: 0.32, bounce: 0.15 }} />}
                        <span className="seg-text">{text}</span>
                    </motion.button>
                );
            })}
        </div>
    );
}

/**
 * Press-and-hold jog button. In "hold" mode it runs while pressed; in "step"
 * mode a tap fires once. Release is watched on the window so letting go
 * outside the button still stops motion.
 */
export function HoldButton({ hold, onStart, onStop, onTap, onEnter, onLeave, className = '', children, ...rest }) {
    const [active, setActive] = useState(false);
    const cleanup = useRef(null);
    useEffect(() => () => cleanup.current?.(), []);

    const down = (e) => {
        if (e.button !== undefined && e.button !== 0) return;
        e.preventDefault();
        const btn = e.currentTarget;
        setActive(true);
        if (hold) onStart();
        const release = (ev) => {
            removeEventListener('pointerup', release);
            removeEventListener('pointercancel', release);
            cleanup.current = null;
            setActive(false);
            if (hold) onStop();
            else if (ev.type === 'pointerup' && btn.contains(ev.target)) onTap();
        };
        addEventListener('pointerup', release);
        addEventListener('pointercancel', release);
        cleanup.current = () => release({ type: 'cancel' });
    };

    return (
        <motion.button
            className={`hold ${active ? 'active' : ''} ${className}`}
            animate={{ scale: active ? 0.94 : 1 }}
            transition={{ type: 'spring', duration: 0.2, bounce: 0.3 }}
            onPointerDown={down}
            onPointerEnter={onEnter}
            onPointerLeave={onLeave}
            onContextMenu={(e) => e.preventDefault()}
            {...rest}
        >
            {children}
        </motion.button>
    );
}

export function Field({ label, children, hint }) {
    return (
        <label className="field">
            <span className="field-label">{label}</span>
            <span className="field-body">{children}</span>
            {hint && <span className="field-hint">{hint}</span>}
        </label>
    );
}

/** Number input that commits on blur/Enter, and doesn't fight the user while focused. */
export function Num({ value, onCommit, step = 1, min, max, width, ...rest }) {
    const [txt, setTxt] = useState(String(value ?? ''));
    const focused = useRef(false);
    useEffect(() => { if (!focused.current) setTxt(String(value ?? '')); }, [value]);
    const commit = () => {
        const v = parseFloat(txt);
        if (Number.isFinite(v)) onCommit(v);
        else setTxt(String(value ?? ''));
    };
    return (
        <input
            type="number" inputMode="decimal" className="num" step={step} min={min} max={max}
            style={width ? { width } : undefined}
            value={txt}
            onFocus={() => { focused.current = true; }}
            onBlur={() => { focused.current = false; commit(); }}
            onChange={(e) => setTxt(e.target.value)}
            onKeyDown={(e) => { if (e.key === 'Enter') e.currentTarget.blur(); }}
            {...rest}
        />
    );
}

export function Section({ title, aside, children }) {
    return (
        <section className="section">
            {(title || aside) && (
                <div className="section-head">
                    <span className="eyebrow">{title}</span>
                    {aside}
                </div>
            )}
            {children}
        </section>
    );
}

export function Badge({ tone = '', children, title }) {
    return <span className={`badge ${tone}`} title={title}>{children}</span>;
}
