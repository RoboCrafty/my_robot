import { useState } from 'react';
import { motion } from 'motion/react';
import { toast } from 'sonner';
import { NJ, cmd, resetSettings, saveSettings, send, setPref, setS, useStore } from '../store.js';
import { TCP_VARIANTS } from '../hud/TargetCard.jsx';
import { Badge, Button, Num, Section, Seg } from '../hud/ui.jsx';

export function SetupPane() {
    const prefs = useStore((s) => s.prefs);
    return (
        <>
            <Robot />

            <Section title="Appearance">
                <div className="kv">
                    <span>Theme</span>
                    <Seg size="sm" label="Theme" value={prefs.theme} onChange={(v) => setPref('theme', v)}
                        items={[['dark', 'Dark'], ['light', 'Light']]} />
                    <span>Rendering</span>
                    <Seg size="sm" label="Rendering quality" value={prefs.quality} onChange={(v) => setPref('quality', v)}
                        items={[['balanced', 'Smooth', 'full frame rate'], ['cinematic', 'Cinematic', 'ambient occlusion + bloom; heavier on the GPU']]} />
                </div>
                <p className="muted small">Stored per device, so a phone can stay on Smooth while a desktop runs Cinematic.</p>
            </Section>

            <Section title="Move-tool control">
                <Seg grow label="TCP control variant" value={prefs.tcp} onChange={(v) => setPref('tcp', v)}
                    items={TCP_VARIANTS.map(([v, l, d]) => [v, l, d])} />
                <ul className="variant-list">
                    {TCP_VARIANTS.map(([v, l, d]) => (
                        <li key={v} className={prefs.tcp === v ? 'on' : ''} onClick={() => setPref('tcp', v)}>
                            <b>{l}</b><span>{d}</span>
                        </li>
                    ))}
                </ul>
            </Section>

            <Section title="Homing">
                <p className="muted small">Re-homes one joint with its limit switch. Homing J5 also re-homes J6 first: the
                    mechanics need J6 homed for J5's switch to be reachable.</p>
                <div className="motors">
                    {Array.from({ length: NJ }, (_, j) => (
                        <motion.button key={j} whileTap={{ scale: 0.94 }} className="motor"
                            onClick={() => { if (confirm(`Re-home J${j + 1} using its limit switch?`)) cmd(`rehome ${j + 1}`); }}>
                            J{j + 1}
                        </motion.button>
                    ))}
                </div>
            </Section>

            <Limits />

            <Increments />

            <Section title="Session settings">
                <p className="muted small">Saves jog mode, frame, speeds and step sizes to <code>poses.json</code>.</p>
                <div className="row2">
                    <Button variant="accent" size="sm" onClick={saveSettings}>Save settings</Button>
                    <Button variant="ghost" size="sm" onClick={resetSettings}>Reset to defaults</Button>
                </div>
            </Section>
        </>
    );
}

const hm = (s) => `${Math.floor(s / 3600)} h ${String(Math.floor(s / 60) % 60).padStart(2, '0')} m`;

// Live driver power / homing per joint, plus the lifetime odometer the ESP32
// keeps in its flash (so it follows the robot, not this computer).
function Robot() {
    const power = useStore((s) => s.st.power);
    const homed = useStore((s) => s.st.homed);
    const block = useStore((s) => s.st.block);
    const sim = useStore((s) => s.st.sim);
    const o = useStore((s) => s.st.odo);
    const powerOff = block.startsWith('motor power');
    const badge = powerOff ? <Badge tone="bad">power off</Badge>
        : block ? <Badge tone="warn">not homed</Badge>
        : <Badge tone="good">ready</Badge>;
    return (
        <Section title="Robot" aside={badge}>
            <div className="motors">
                {power.map((on, j) => {
                    const state = !on ? 'bad' : homed[j] ? 'on' : 'warn';
                    const tip = !on ? 'driver not answering — no motor power' : homed[j] ? 'powered and homed' : 'powered, needs homing';
                    return <span key={j} className={`motor ${state}`} title={tip}><span className="motor-led" />J{j + 1}</span>;
                })}
            </div>
            {o ? (
                <div className="kv">
                    <span>Power-on</span><span className="mono">{hm(o.power_s)}</span>
                    <span>Motors energized</span><span className="mono">{hm(o.servo_s)}</span>
                    <span>Moving</span><span className="mono">{hm(o.motion_s)}</span>
                    <span>Power cycles</span><span className="mono">{o.power_cycles}</span>
                    <span>Homings</span><span className="mono">{o.homings}</span>
                    <span>Driver faults</span><span className="mono">{o.faults}</span>
                    <span>Travel</span>
                    <span className="mono small">{o.travel_deg.map((d, j) => `J${j + 1} ${(d / 360).toFixed(1)}`).join('  ')} rev</span>
                </div>
            ) : (
                <p className="muted small">{sim ? 'Lifetime counters live on the ESP32; not available in the simulator.'
                    : 'Waiting for the ESP32 odometer…'}</p>
            )}
        </Section>
    );
}

function Limits() {
    const st = useStore((s) => s.st);
    const [edit, setEdit] = useState({});
    const val = (k, j) => edit[`${k}${j}`] ?? st[{ v: 'vmax', a: 'amax', k: 'jmax' }[k]][j];
    const apply = () => {
        for (let j = 0; j < NJ; j++) {
            for (const [k, verb] of [['v', 'vel'], ['a', 'acc'], ['k', 'jerk']]) {
                const v = edit[`${k}${j}`];
                if (v !== undefined) cmd(`${verb} ${j + 1} ${v}`);
            }
        }
        setEdit({});
        toast.success('Limits applied');
    };
    return (
        <Section title="Motion limits">
            <table className="limits">
                <thead><tr><th>J</th><th>vel °/s</th><th>acc °/s²</th><th>jerk °/s³</th></tr></thead>
                <tbody>
                    {Array.from({ length: NJ }, (_, j) => (
                        <tr key={j}>
                            <td>J{j + 1}</td>
                            {[['v', 1], ['a', 10], ['k', 10]].map(([k, step]) => (
                                <td key={k}><Num value={val(k, j)} step={step} onCommit={(v) => setEdit((e) => ({ ...e, [`${k}${j}`]: v }))} /></td>
                            ))}
                        </tr>
                    ))}
                </tbody>
            </table>
            <Button variant={Object.keys(edit).length ? 'accent' : 'default'} size="sm" onClick={apply}>Apply limits</Button>
        </Section>
    );
}

function Increments() {
    const increments = useStore((s) => s.config.increments) || [];
    const [v, setV] = useState('');
    const add = () => {
        const n = parseFloat(v);
        if (!Number.isFinite(n) || n <= 0) return;
        send({ type: 'set_increments', values: [...new Set([...increments, n])].sort((a, b) => a - b) });
        setS({ selInc: n });
        setV('');
    };
    return (
        <Section title="Joint step presets">
            <div className="tool-row">
                <span className="muted small">{increments.map((x) => `${x}°`).join(' · ')}</span>
                <span className="spacer" />
                <input className="num" type="number" placeholder="deg" value={v} onChange={(e) => setV(e.target.value)}
                    onKeyDown={(e) => { if (e.key === 'Enter') add(); }} style={{ width: 64 }} />
                <Button size="sm" onClick={add}>Add</Button>
            </div>
        </Section>
    );
}
