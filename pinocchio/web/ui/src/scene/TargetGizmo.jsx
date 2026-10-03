import * as THREE from 'three';
import { useLayoutEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Billboard, PivotControls } from '@react-three/drei';
import { commitPlan, live, useStore } from '../store.js';
import { ChainIK } from './ik.js';
import { ACCENT, AXIS_COLORS, JOINT_NAMES, TIP_LINK, jointPath } from './urdf.js';
import { haloTexture, makeLine, setLine } from './lines.js';

const DEG = Math.PI / 180;
const BAD = new THREE.Color('#ff3b4a');
const GOOD = new THREE.Color(ACCENT);

/** Lets the HUD reset the target without owning any three.js state. */
export const targetApi = { reset: () => {} };

const _m = new THREE.Matrix4();
const _inv = new THREE.Matrix4();
const _p = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _s = new THREE.Vector3();
const _tp = new THREE.Vector3();
const _tq = new THREE.Quaternion();
const _e = new THREE.Euler();
const _plane = new THREE.Plane();
const _hit = new THREE.Vector3();

/**
 * Ghost-target TCP control. Variants (prefs.tcp):
 *   a  gizmo, confirm with a button
 *   b  gizmo, moves when the drag is released
 *   d  free drag of a bead in the view plane (Shift = vertical only), confirm
 */
export function TargetGizmo({ models, zupRef }) {
    const { robot, ghost, scratch } = models;
    const variant = useStore((s) => s.prefs.tcp);
    const preview = useStore((s) => s.preview);
    const controls = useThree((s) => s.controls);
    const camera = useThree((s) => s.camera);

    // Target pose in the arm's base frame (URDF coords) -- exactly what the
    // controller's `move`/`movel` expect, so no frame juggling on commit.
    const matrix = useMemo(() => new THREE.Matrix4(), []);
    const ik = useMemo(() => new ChainIK(ghost, JOINT_NAMES, TIP_LINK), [ghost]);
    const seed = useRef(new Array(6).fill(0));
    const reach = useRef(true);
    const lastPath = useRef(0);
    const lastPush = useRef(0);
    const bead = useRef();
    const paths = useMemo(() => ({
        j: makeLine({ color: ACCENT, linewidth: 2.5, dashed: true, dashSize: 0.008, gapSize: 0.006, opacity: 0.9, depthTest: false, toneMapped: false }),
        l: makeLine({ color: '#f5f5f7', linewidth: 2.5, dashed: true, dashSize: 0.008, gapSize: 0.006, opacity: 0.8, depthTest: false, toneMapped: false }),
        drop: makeLine({ color: ACCENT, linewidth: 1.2, dashed: true, dashSize: 0.004, gapSize: 0.004, opacity: 0.55, depthTest: false, toneMapped: false }),
    }), []);

    function solve() {
        const zup = zupRef.current;
        zup.updateMatrixWorld(true);
        _m.multiplyMatrices(zup.matrixWorld, matrix);
        const r = ik.solve(_m, seed.current);
        if (r.ok) seed.current = r.q;
        // A failed solve leaves the chain wherever the iteration gave up, often
        // a contorted pose. Park the ghost on the last reachable one instead.
        else seed.current.forEach((v, i) => ghost.joints[JOINT_NAMES[i]].setJointValue(v));
        reach.current = r.ok;
        ghost.updateMatrixWorld(true);
        matrix.decompose(_p, _q, _s);
        _e.setFromQuaternion(_q, 'ZYX');   // controller speaks Pinocchio RPY = ZYX
        setLine(paths.drop, [_p.x, _p.y, _p.z, _p.x, _p.y, 0]);
        const prev = useStore.getState().plan;
        useStore.setState({
            plan: {
                pos: _p.toArray(), rpy: [_e.x, _e.y, _e.z], reachable: r.ok,
                dist: prev?.dist ?? 0, ang: prev?.ang ?? 0,
            },
        });
        updatePaths();
    }

    // Joint moves curve through space; linear moves don't. FK along the joint
    // interpolation shows which one the user is about to get. Paths live in the
    // Z-up group, i.e. in URDF coordinates, same as `scratch`.
    function updatePaths() {
        const q1 = seed.current.map((v) => v / DEG);
        const flat = jointPath(scratch, live.q, q1, 40);
        setLine(paths.j, flat);
        const n = flat.length;
        setLine(paths.l, [flat[0], flat[1], flat[2], flat[n - 3], flat[n - 2], flat[n - 1]]);
    }

    function reset() {
        const zup = zupRef.current;
        zup.updateMatrixWorld(true);
        robot.links[TIP_LINK].updateWorldMatrix(true, false);
        _inv.copy(zup.matrixWorld).invert();
        matrix.multiplyMatrices(_inv, robot.links[TIP_LINK].matrixWorld);
        seed.current = live.q.map((v) => v * DEG);
        solve();
    }
    targetApi.reset = reset;
    targetApi.matrix = matrix;
    targetApi.solve = solve;

    useLayoutEffect(() => {
        reset();
        return () => useStore.setState({ plan: null });
    }, []); // eslint-disable-line react-hooks/exhaustive-deps

    const dragStart = () => { if (controls) controls.enabled = false; };
    const dragEnd = () => {
        if (controls) controls.enabled = true;
        if (variant === 'b') {
            const p = useStore.getState().plan;
            if (p && p.reachable) commitPlan(useStore.getState().prefs.autoMotion === 'l' ? 'movel' : 'move');
        }
    };

    // ---- variant d: drag the bead across a camera-facing plane
    const drag = useRef(null);
    const onBeadDown = (e) => {
        e.stopPropagation();
        e.target.setPointerCapture(e.pointerId);
        dragStart();
        const zup = zupRef.current;
        _p.setFromMatrixPosition(matrix).applyMatrix4(zup.matrixWorld);
        const n = camera.getWorldDirection(new THREE.Vector3());
        const vertical = e.shiftKey;
        if (vertical) { n.y = 0; n.normalize(); }
        _plane.setFromNormalAndCoplanarPoint(n, _p);
        drag.current = { vertical, start: _p.clone() };
    };
    const onBeadMove = (e) => {
        if (!drag.current) return;
        e.stopPropagation();
        if (!e.ray.intersectPlane(_plane, _hit)) return;
        if (drag.current.vertical) { _hit.x = drag.current.start.x; _hit.z = drag.current.start.z; }
        _inv.copy(zupRef.current.matrixWorld).invert();
        _hit.applyMatrix4(_inv);
        matrix.setPosition(_hit);
        solve();
    };
    const onBeadUp = (e) => {
        if (!drag.current) return;
        e.target.releasePointerCapture?.(e.pointerId);
        drag.current = null;
        dragEnd();
    };

    useFrame(({ clock, size }, dt) => {
        const mat = ghost.userData.material;
        // How far the live tool still is from the target. The ghost fades as the
        // arm walks into it, so "arrived" needs no extra UI.
        robot.links[TIP_LINK].updateWorldMatrix(true, false);
        robot.links[TIP_LINK].matrixWorld.decompose(_p, _q, _s);
        _m.multiplyMatrices(zupRef.current.matrixWorld, matrix).decompose(_tp, _tq, _s);
        const dist = _p.distanceTo(_tp) * 1000;
        const ang = _q.angleTo(_tq) / DEG;
        const show = dist > 1.5 || ang > 0.5;
        const u = mat.uniforms;
        u.uOpacity.value = THREE.MathUtils.damp(u.uOpacity.value, show ? 1 : 0, 10, dt);
        u.uColor.value.lerp(reach.current ? GOOD : BAD, 1 - Math.exp(-14 * dt));
        ghost.visible = u.uOpacity.value > 0.01;

        for (const k of ['j', 'l', 'drop']) {
            const l = paths[k];
            l.material.resolution.set(size.width, size.height);
            l.material.dashOffset -= dt * 0.03;
        }
        paths.j.visible = show && reach.current && preview === 'j';
        paths.l.visible = show && reach.current && preview === 'l';
        paths.drop.visible = true;
        if (bead.current) bead.current.position.setFromMatrixPosition(matrix);

        const now = clock.elapsedTime;
        if (show && now - lastPath.current > 0.12) {
            lastPath.current = now;
            updatePaths();
        }
        if (now - lastPush.current > 0.1) {
            lastPush.current = now;
            const plan = useStore.getState().plan;
            if (plan && (Math.abs(plan.dist - dist) > 0.05 || Math.abs(plan.ang - ang) > 0.05)) {
                useStore.setState({ plan: { ...plan, dist, ang } });
            }
        }
    });

    return (
        <>
            {variant !== 'd' ? (
                <PivotControls
                    matrix={matrix}
                    autoTransform={false}
                    fixed
                    scale={110}
                    lineWidth={2.5}
                    depthTest={false}
                    disableScaling
                    annotations
                    annotationsClass="gizmo-annot"
                    axisColors={AXIS_COLORS}
                    hoveredColor={ACCENT}
                    onDragStart={dragStart}
                    onDragEnd={dragEnd}
                    onDrag={(l) => { matrix.copy(l); solve(); }}
                >
                    <TargetBead />
                </PivotControls>
            ) : (
                <group ref={bead}>
                    <mesh
                        onPointerDown={onBeadDown} onPointerMove={onBeadMove}
                        onPointerUp={onBeadUp} onPointerCancel={onBeadUp}
                        onPointerOver={() => { document.body.style.cursor = 'grab'; }}
                        onPointerOut={() => { document.body.style.cursor = ''; }}
                    >
                        <sphereGeometry args={[0.03, 12, 12]} />
                        <meshBasicMaterial visible={false} />
                    </mesh>
                    <TargetBead big />
                </group>
            )}
            <primitive object={paths.j} />
            <primitive object={paths.l} />
            <primitive object={paths.drop} />
        </>
    );
}

function TargetBead({ big }) {
    const core = useMemo(() => new THREE.Color(ACCENT).multiplyScalar(4), []);
    return (
        <>
            <mesh renderOrder={6}>
                <sphereGeometry args={[big ? 0.008 : 0.005, 20, 20]} />
                <meshBasicMaterial color={core} toneMapped={false} depthTest={false} />
            </mesh>
            <sprite scale={big ? 0.07 : 0.045} renderOrder={6}>
                <spriteMaterial map={haloTexture()} color={ACCENT} transparent opacity={0.5}
                    depthWrite={false} depthTest={false} toneMapped={false} />
            </sprite>
            {big && (
                <Billboard>
                    <mesh renderOrder={6}>
                        <ringGeometry args={[0.016, 0.018, 48]} />
                        <meshBasicMaterial color={ACCENT} transparent opacity={0.8} depthTest={false} toneMapped={false} />
                    </mesh>
                </Billboard>
            )}
        </>
    );
}
