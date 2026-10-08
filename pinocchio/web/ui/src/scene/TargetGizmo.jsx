import * as THREE from 'three';
import { useEffect, useLayoutEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Billboard, PivotControls } from '@react-three/drei';
import { commitPlan, live, useStore } from '../store.js';
import { ChainIK } from './ik.js';
import { ACCENT, AXIS_COLORS, JOINT_NAMES, TIP_LINK, fkPos, jointPath } from './urdf.js';
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
    const invalidate = useThree((s) => s.invalidate);
    useEffect(() => { invalidate(); }, [preview, variant, invalidate]);
    const camera = useThree((s) => s.camera);

    // Target pose in the arm's base frame (URDF coords) -- exactly what the
    // controller's `move`/`movel` expect, so no frame juggling on commit.
    const matrix = useMemo(() => new THREE.Matrix4(), []);
    const ik = useMemo(() => new ChainIK(ghost, JOINT_NAMES, TIP_LINK), [ghost]);
    const seed = useRef(new Array(6).fill(0));      // ghost pose, rad
    const verdict = useRef({ jointOk: true, linOk: true, linWhy: null, qJoint: null });
    const checkedQ = useRef(null);
    const lastCheck = useRef(0);
    const lastPush = useRef(0);
    const bead = useRef();
    const paths = useMemo(() => ({
        j: makeLine({ color: ACCENT, linewidth: 2.5, dashed: true, dashSize: 0.008, gapSize: 0.006, opacity: 0.9, depthTest: false, toneMapped: false }),
        l: makeLine({ color: '#f5f5f7', linewidth: 2.5, dashed: true, dashSize: 0.008, gapSize: 0.006, opacity: 0.8, depthTest: false, toneMapped: false }),
        drop: makeLine({ color: ACCENT, linewidth: 1.2, dashed: true, dashSize: 0.004, gapSize: 0.004, opacity: 0.55, depthTest: false, toneMapped: false }),
    }), []);

    const targetWorld = () => {
        zupRef.current.updateMatrixWorld(true);
        return new THREE.Matrix4().multiplyMatrices(zupRef.current.matrixWorld, matrix);
    };

    // The controller's `movel` follows the straight line (lerp position, slerp
    // rotation) by resolved rate from the current pose. It aborts if any point
    // has no IK solution, or if the path direction becomes >10% unachievable
    // near a singularity (main.cpp, Mode::CartLin). Walk the same line.
    function checkLinear(T, q0) {
        ik.setQ(q0);
        ik.tip.matrixWorld.decompose(_p, _q, _s);
        T.decompose(_tp, _tq, _s);
        const lin = _p.distanceTo(_tp), ang = _q.angleTo(_tq);
        if (lin < 1e-6 && ang < 1e-6) return { linOk: true, linWhy: null };
        // Path direction as the controller builds it: [dp; world rotation vector].
        const dr = _tq.clone().multiply(_q.clone().invert());
        if (dr.w < 0) dr.set(-dr.x, -dr.y, -dr.z, -dr.w);
        const sn = Math.hypot(dr.x, dr.y, dr.z), k = sn < 1e-9 ? 0 : 2 * Math.atan2(sn, dr.w) / sn;
        const twist = [_tp.x - _p.x, _tp.y - _p.y, _tp.z - _p.z, dr.x * k, dr.y * k, dr.z * k];

        const steps = Math.min(60, Math.max(2, Math.ceil(Math.max(lin / 0.005, ang / 0.05))));
        const M = new THREE.Matrix4(), P = new THREE.Vector3(), Q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
        const sp = _p.clone(), sq = _q.clone();
        let q = q0;
        let worst = 0;
        for (let i = 0; i <= steps; i++) {
            const s = i / steps;
            if (i > 0) {
                M.compose(P.lerpVectors(sp, _tp, s), Q.slerpQuaternions(sq, _tq, s), one);
                const r = ik.solve(M, q, { maxIters: 40 });
                if (!r.ok) return { linOk: false, linWhy: 'Line leaves the workspace or hits a joint limit' };
                // A big joint jump over one short step means the line crosses a
                // singularity the arm would have to flip through.
                if (r.q.some((v, j) => Math.abs(v - q[j]) > 0.35)) return { linOk: false, linWhy: 'Line passes through a singularity' };
                q = r.q;
            } else {
                ik.setQ(q);
            }
            worst = Math.max(worst, ik.trackErr(twist).trackErr);
            if (worst > 0.1) return { linOk: false, linWhy: 'Line runs into a singularity' };
        }
        return { linOk: true, linWhy: worst > 0.05 ? 'Passes near a singularity; may slow down' : null };
    }

    // Solve from the arm's CURRENT joints with the controller's IK settings, so
    // "reachable" means the controller will really accept the move.
    function check() {
        const T = targetWorld();
        matrix.decompose(_p, _q, _s);
        const opts = { reachPos: [_p.x, _p.y, _p.z], maxReach: models.maxReach };
        const q0 = live.q.map((v) => v * DEG);
        checkedQ.current = live.q.slice();
        const rj = ik.solve(T, q0, opts);
        const lin = opts.maxReach && Math.hypot(...opts.reachPos) > opts.maxReach
            ? { linOk: false, linWhy: 'Beyond the arm\'s reach' } : checkLinear(T, q0);
        verdict.current = { jointOk: rj.ok, qJoint: rj.ok ? rj.q : null, ...lin };
        // The ghost shows the controller's own solution when there is one;
        // otherwise the nearest pose reachable from the previous drag position.
        if (rj.ok) seed.current = rj.q;
        else {
            const rs = ik.solve(T, seed.current, opts);
            if (rs.ok) seed.current = rs.q;
        }
        ik.setQ(seed.current);
    }

    function publish() {
        const v = verdict.current;
        matrix.decompose(_p, _q, _s);
        _e.setFromQuaternion(_q, 'ZYX');   // controller speaks Pinocchio RPY = ZYX
        const prev = useStore.getState().plan;
        useStore.setState({
            plan: {
                pos: _p.toArray(), rpy: [_e.x, _e.y, _e.z],
                jointOk: v.jointOk, linOk: v.linOk, linWhy: v.linWhy, reachable: v.jointOk || v.linOk,
                dist: prev?.dist ?? 0, ang: prev?.ang ?? 0,
            },
        });
    }

    function solve() {
        invalidate();
        check();
        matrix.decompose(_p, _q, _s);
        setLine(paths.drop, [_p.x, _p.y, _p.z, _p.x, _p.y, 0]);
        publish();
        updatePaths();
    }

    // Joint moves curve through space; linear moves don't. FK along the joint
    // interpolation shows which one the user is about to get. Paths live in the
    // Z-up group, i.e. in URDF coordinates, same as `scratch`.
    function updatePaths() {
        const v = verdict.current;
        if (v.qJoint) setLine(paths.j, jointPath(scratch, live.q, v.qJoint.map((x) => x / DEG), 40));
        const a = fkPos(scratch, live.q);
        matrix.decompose(_p, _q, _s);
        setLine(paths.l, [a.x, a.y, a.z, _p.x, _p.y, _p.z]);
        paths.l.material.color.set(v.linOk ? '#f5f5f7' : '#ff3b4a');
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
        if (variant === 'b') commitPlan(useStore.getState().prefs.autoMotion === 'l' ? 'movel' : 'move');
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

    useFrame(({ clock, size, invalidate }, dt) => {
        const mat = ghost.userData.material;
        // How far the live tool still is from the target. The ghost fades as the
        // arm walks into it, so "arrived" needs no extra UI.
        robot.links[TIP_LINK].updateWorldMatrix(true, false);
        robot.links[TIP_LINK].matrixWorld.decompose(_p, _q, _s);
        _m.multiplyMatrices(zupRef.current.matrixWorld, matrix).decompose(_tp, _tq, _s);
        const dist = _p.distanceTo(_tp) * 1000;
        const ang = _q.angleTo(_tq) / DEG;
        const show = dist > 1.5 || ang > 0.5;
        const v = verdict.current;
        const u = mat.uniforms;
        const goalOp = show ? 1 : 0;
        const goalCol = v.jointOk || v.linOk ? GOOD : BAD;
        u.uOpacity.value = THREE.MathUtils.damp(u.uOpacity.value, goalOp, 10, dt);
        u.uColor.value.lerp(goalCol, 1 - Math.exp(-14 * dt));
        ghost.visible = u.uOpacity.value > 0.01;
        if (Math.abs(u.uOpacity.value - goalOp) > 0.005 || !u.uColor.value.equals(goalCol)) invalidate();

        for (const k of ['j', 'l', 'drop']) paths[k].material.resolution.set(size.width, size.height);
        paths.j.visible = show && !!v.qJoint && preview === 'j';
        paths.l.visible = show && preview === 'l';
        paths.drop.visible = true;
        if (bead.current) bead.current.position.setFromMatrixPosition(matrix);

        // Reachability is judged from where the arm is now, so re-judge as it moves.
        const now = clock.elapsedTime;
        if (show && now - lastCheck.current > 0.15 && checkedQ.current
            && live.q.some((x, i) => Math.abs(x - checkedQ.current[i]) > 0.05)) {
            lastCheck.current = now;
            check();
            publish();
            updatePaths();
        }
        const plan = useStore.getState().plan;
        if (plan && (Math.abs(plan.dist - dist) > 0.05 || Math.abs(plan.ang - ang) > 0.05)) {
            if (now - lastPush.current > 0.1) {
                lastPush.current = now;
                useStore.setState({ plan: { ...plan, dist, ang } });
            } else {
                invalidate();   // throttled: come back for the final value
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
