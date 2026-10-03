import * as THREE from 'three';
import { useEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { R2D, cartJogVel, stopEverything, useStore } from '../store.js';
import { AXIS_COLORS, TIP_LINK } from './urdf.js';

const DRAG_FULL_SCALE_PX = 120;   // pixels of drag that map to 100% jog speed
const L = 0.055, R = 0.082;       // rings sit outside the arrow tips so picking is unambiguous
const _v = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _pq = new THREE.Quaternion();

/**
 * Variant c: velocity handles. Drag an arrow or ring and the arm jogs live;
 * the farther you drag, the faster it goes. Release stops it.
 */
export function RateHandles({ robot }) {
    const g = useRef();
    const frame = useStore((s) => s.S.cartFrame);
    const camera = useThree((s) => s.camera);
    const controls = useThree((s) => s.controls);
    const size = useThree((s) => s.size);
    const tip = robot.links[TIP_LINK];
    const drag = useRef(null);

    const handles = useMemo(() => AXIS_COLORS.flatMap((c, a) => [
        { axis: a, kind: 'lin', mat: new THREE.MeshBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.95, toneMapped: false }) },
        { axis: 3 + a, kind: 'rot', mat: new THREE.MeshBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.55, toneMapped: false }) },
    ]), []);
    const refs = useRef({});
    const hoverAxis = useStore((s) => s.hoverAxis);
    const invalidate = useThree((s) => s.invalidate);
    useEffect(() => { invalidate(); }, [hoverAxis, invalidate]);

    useFrame((state, dt) => {
        tip.getWorldPosition(_v);
        g.current.parent.worldToLocal(g.current.position.copy(_v));
        if (frame === 'tool') {
            tip.getWorldQuaternion(_q);
            g.current.parent.getWorldQuaternion(_pq);
            g.current.quaternion.copy(_pq.invert().multiply(_q));
        } else {
            g.current.quaternion.identity();
        }
        const hover = useStore.getState().hoverAxis;
        const d = drag.current;
        let settling = false;
        for (const h of handles) {
            const o = refs.current[h.axis];
            if (!o) continue;
            const active = d ? d.axis === h.axis : hover === h.axis;
            const dim = (d || hover != null) && !active;
            const mag = d && d.axis === h.axis ? Math.min(1, Math.abs(d.rate)) : 0;
            const base = h.kind === 'rot' ? 0.55 : 0.95;
            const op = dim ? base * 0.2 : active ? 1 : base;
            h.mat.opacity = THREE.MathUtils.damp(h.mat.opacity, op, 16, dt);
            // Grow with the commanded rate so a short drag visibly reads as slow.
            const goal = h.kind === 'lin' ? 1 + mag * 0.9 : 1 + mag * 0.3 + (active ? 0.06 : 0);
            const sy = THREE.MathUtils.damp(o.scale.y, goal, 16, dt);
            if (h.kind === 'lin') o.scale.set(1, sy, 1);
            else o.scale.setScalar(sy);
            if (Math.abs(sy - goal) > 0.002 || Math.abs(h.mat.opacity - op) > 0.005) settling = true;
        }
        if (settling) state.invalidate();
    });

    const onDown = (h) => (e) => {
        e.stopPropagation();
        e.target.setPointerCapture(e.pointerId);
        if (controls) controls.enabled = false;
        // Screen-space direction the handle moves in: the axis itself for an
        // arrow, the tangent at the grab point for a ring.
        const origin = g.current.getWorldPosition(new THREE.Vector3());
        const axisDir = new THREE.Vector3().setFromMatrixColumn(g.current.matrixWorld, h.axis % 3).normalize();
        const dir3 = h.kind === 'lin' ? axisDir
            : new THREE.Vector3().subVectors(e.point, origin).cross(axisDir).normalize().negate();
        const a = origin.clone().project(camera);
        const b = origin.clone().addScaledVector(dir3, 0.05).project(camera);
        const screen = new THREE.Vector2((b.x - a.x) * size.width, -(b.y - a.y) * size.height);
        if (screen.lengthSq() < 1e-8) screen.set(1, 0);
        screen.normalize();
        drag.current = { axis: h.axis, screen, x0: e.clientX, y0: e.clientY, rate: 0 };
        document.body.style.cursor = 'grabbing';
    };
    const onMove = (e) => {
        const d = drag.current;
        if (!d) return;
        const px = (e.clientX - d.x0) * d.screen.x + (e.clientY - d.y0) * d.screen.y;
        const rate = Math.max(-1, Math.min(1, px / DRAG_FULL_SCALE_PX));
        if (Math.abs(rate - d.rate) < 0.02) return;
        d.rate = rate;
        invalidate();
        const S = useStore.getState().S;
        cartJogVel(d.axis, rate * (d.axis < 3 ? S.cartLinSpeed : S.cartAngSpeed / R2D));
        useStore.setState({ jogHud: { axis: d.axis, rate } });
    };
    const onUp = (e) => {
        if (!drag.current) return;
        e.target.releasePointerCapture?.(e.pointerId);
        drag.current = null;
        if (controls) controls.enabled = true;
        document.body.style.cursor = '';
        stopEverything();
    };

    return (
        <group ref={g} renderOrder={10}>
            {handles.map((h) => {
                const a = h.axis % 3;
                const events = {
                    onPointerDown: onDown(h), onPointerMove: onMove, onPointerUp: onUp, onPointerCancel: onUp,
                    onPointerOver: (e) => { e.stopPropagation(); if (!drag.current) document.body.style.cursor = 'grab'; },
                    onPointerOut: () => { if (!drag.current) document.body.style.cursor = ''; },
                };
                if (h.kind === 'lin') {
                    // Cylinders are along +Y; rotate each onto its axis.
                    const rot = a === 0 ? [0, 0, -Math.PI / 2] : a === 2 ? [Math.PI / 2, 0, 0] : [0, 0, 0];
                    return (
                        <group key={h.axis} rotation={rot} ref={(o) => { refs.current[h.axis] = o; }}>
                            <mesh material={h.mat} renderOrder={10}><cylinderGeometry args={[0.0022, 0.0022, 2 * L, 10]} /></mesh>
                            {[-1, 1].map((s) => (
                                <mesh key={s} material={h.mat} renderOrder={10} position-y={s * L} rotation-x={s > 0 ? 0 : Math.PI}>
                                    <coneGeometry args={[0.0085, 0.022, 16]} />
                                </mesh>
                            ))}
                            <mesh {...events}>
                                <cylinderGeometry args={[0.014, 0.014, 2 * L + 0.04, 8]} />
                                <meshBasicMaterial visible={false} />
                            </mesh>
                        </group>
                    );
                }
                const rot = a === 0 ? [0, Math.PI / 2, 0] : a === 1 ? [Math.PI / 2, 0, 0] : [0, 0, 0];
                return (
                    <group key={h.axis} rotation={rot} ref={(o) => { refs.current[h.axis] = o; }}>
                        <mesh material={h.mat} renderOrder={10}><torusGeometry args={[R, 0.0028, 8, 96]} /></mesh>
                        <mesh {...events}>
                            <torusGeometry args={[R, 0.012, 6, 48]} />
                            <meshBasicMaterial visible={false} />
                        </mesh>
                    </group>
                );
            })}
        </group>
    );
}
