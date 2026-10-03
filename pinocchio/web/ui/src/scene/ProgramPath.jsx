import * as THREE from 'three';
import { useEffect, useMemo, useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import { useStore } from '../store.js';
import { programMoves } from '../program.js';
import { ACCENT, fkPos, jointPath } from './urdf.js';
import { makeLine, setLine } from './lines.js';

/**
 * While the Points or Program tab is open: every waypoint as a labelled dot,
 * and the TCP path the current program will trace. Rendered in the Z-up group,
 * so everything is in URDF coordinates, same as the FK scratch model.
 */
export function ProgramPath({ scratch }) {
    const tab = useStore((s) => s.S.tab);
    const poses = useStore((s) => s.config.poses);
    const nodes = useStore((s) => s.editor.nodes);
    const sel = useStore((s) => s.editor.sel);
    const runNode = useStore((s) => s.prog.node);
    const hoverWp = useStore((s) => s.hoverWp);
    const light = useStore((s) => s.prefs.theme) === 'light';
    const active = tab === 'points' || tab === 'program';

    const lines = useMemo(() => ({
        path: makeLine({ vertexColors: true, linewidth: 3, dashed: true, dashSize: 0.01, gapSize: 0.006, depthTest: false, opacity: 0.9, toneMapped: false }),
        seg: makeLine({ color: ACCENT, linewidth: 4, depthTest: false, toneMapped: false }),
    }), []);
    const [dots, setDots] = useState([]);
    const segOf = useRef({});

    useEffect(() => {
        if (!active) { lines.path.visible = lines.seg.visible = false; setDots([]); return; }
        const moves = tab === 'program' ? programMoves(nodes) : [];
        const used = new Set(moves.map((m) => m.wp));
        const pts = [];
        for (const [name, p] of Object.entries(poses)) {
            if (tab === 'program' && !used.has(name)) continue;
            const a = (Array.isArray(p) ? p : p?.angles) || [];
            if (a.length === 6) pts.push({ name, pos: fkPos(scratch, a.map(Number)).toArray() });
        }
        setDots(pts);

        if (tab !== 'program') { lines.path.visible = lines.seg.visible = false; return; }
        const flat = [], colors = [];
        segOf.current = {};
        const c0 = new THREE.Color('#ffd2b3'), c1 = new THREE.Color(ACCENT), c = new THREE.Color();
        for (let i = 1; i < moves.length; i++) {
            const a = moves[i - 1], b = moves[i];
            const seg = b.motion === 'l'
                ? [...fkPos(scratch, a.q).toArray(), ...fkPos(scratch, b.q).toArray()]
                : jointPath(scratch, a.q, b.q, 24);
            segOf.current[b.id] = seg;
            flat.push(...seg);
        }
        // Gradient from start (pale) to end (accent) shows direction at a glance.
        const n = flat.length / 3;
        for (let i = 0; i < n; i++) {
            c.copy(c0).lerp(c1, n > 1 ? i / (n - 1) : 1);
            colors.push(c.r, c.g, c.b);
        }
        if (n >= 2) { setLine(lines.path, flat, colors); lines.path.visible = true; }
        else lines.path.visible = false;
    }, [active, tab, poses, nodes, scratch, lines]);

    // Highlight the running step, else the selected one.
    useEffect(() => {
        const seg = active && segOf.current[runNode || sel];
        if (seg) { setLine(lines.seg, seg); lines.seg.visible = true; }
        else lines.seg.visible = false;
    }, [active, sel, runNode, nodes, poses, lines]);

    useFrame(({ size }, dt) => {
        lines.path.material.resolution.set(size.width, size.height);
        lines.seg.material.resolution.set(size.width, size.height);
        lines.path.material.dashOffset -= dt * 0.04;
    });

    if (!active) return null;
    // On the Points tab every waypoint is a dot but only the hovered one gets a
    // label; dozens of labels at once are unreadable.
    const labelAll = tab === 'program';
    const dot = light ? '#3a3a40' : '#ffffff';
    return (
        <>
            <primitive object={lines.path} />
            <primitive object={lines.seg} />
            {dots.map((d) => (
                <group key={d.name} position={d.pos}>
                    <mesh renderOrder={4} scale={hoverWp === d.name ? 1.8 : 1}>
                        <sphereGeometry args={[0.0045, 16, 16]} />
                        <meshBasicMaterial color={hoverWp === d.name ? ACCENT : dot} depthTest={false} toneMapped={false} />
                    </mesh>
                    {(labelAll || hoverWp === d.name) && (
                        <Html center style={{ pointerEvents: 'none' }} zIndexRange={[5, 0]}>
                            <div className={`wp-label ${hoverWp === d.name ? 'on' : ''}`}>{d.name}</div>
                        </Html>
                    )}
                </group>
            ))}
        </>
    );
}
