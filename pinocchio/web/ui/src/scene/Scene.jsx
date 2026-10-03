import * as THREE from 'three';
import { useEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Billboard, CameraControls, ContactShadows, Environment, Grid, Lightformer } from '@react-three/drei';
import { Bloom, EffectComposer, N8AO, ToneMapping, Vignette } from '@react-three/postprocessing';
import { ToneMappingMode } from 'postprocessing';
import { live, useStore } from '../store.js';
import { ACCENT, JAW_JOINTS, JOINT_NAMES, TIP_LINK, useUrdf } from './urdf.js';
import { haloTexture, makeLine, setLine } from './lines.js';
import { TargetGizmo } from './TargetGizmo.jsx';
import { RateHandles } from './RateHandles.jsx';
import { ProgramPath } from './ProgramPath.jsx';

const DEG = Math.PI / 180;
const GRIP_MAX = 140;
export const PANEL_W = 392;
const _v = new THREE.Vector3();

export const THEMES = {
    dark: { bg: '#0a0a0c', cell: '#232428', section: '#38393f', shadow: 0.42, additive: true },
    light: { bg: '#e8e6e1', cell: '#d2cfc8', section: '#b9b5ad', shadow: 0.2, additive: false },
};

// Camera directions in the scene's Y-up world (the arm itself is Z-up, see Scene).
const VIEWS = {
    iso: [0.62, 0.42, 0.68], front: [0, 0.16, 1], side: [1, 0.16, 0], top: [0.001, 1, 0.02],
};

export function Scene() {
    const urdf = useStore((s) => s.urdf);
    const theme = THEMES[useStore((s) => s.prefs.theme)] || THEMES.dark;
    const cinematic = useStore((s) => s.prefs.quality) === 'cinematic';
    const mode = useStore((s) => s.mode);
    const tcpMode = useStore((s) => s.prefs.tcp);
    const models = useUrdf(urdf);
    const zup = useRef();

    return (
        <>
            <color attach="background" args={[theme.bg]} />
            <fog attach="fog" args={[theme.bg, 2.4, 5.5]} />
            <Studio cinematic={cinematic} />
            <Grid
                infiniteGrid cellSize={0.05} sectionSize={0.25} cellThickness={0.6} sectionThickness={1}
                cellColor={theme.cell} sectionColor={theme.section} fadeDistance={3.2} fadeStrength={2.5}
            />
            {cinematic
                ? <ContactShadows position={[0, 0.0005, 0]} scale={1.6} blur={2.4} far={0.9} opacity={theme.shadow * 1.8} resolution={1024} />
                : (
                    <mesh rotation-x={-Math.PI / 2} position-y={0.0005} receiveShadow>
                        <planeGeometry args={[4, 4]} />
                        <shadowMaterial opacity={theme.shadow} />
                    </mesh>
                )}

            {/* URDF is Z-up; the rest of the scene (drei helpers, camera) is Y-up. */}
            <group ref={zup} rotation-x={-Math.PI / 2}>
                {models && <Arm models={models} />}
                {models && mode === 'tcp' && tcpMode !== 'c' && <TargetGizmo models={models} zupRef={zup} />}
                {models && mode === 'tcp' && tcpMode === 'c' && <RateHandles robot={models.robot} />}
                {models && <ProgramPath scratch={models.scratch} />}
            </group>
            {models && <TcpOrb robot={models.robot} theme={theme} />}
            {models && <Trail robot={models.robot} theme={theme} />}
            {models && mode !== 'tcp' && <GhostOff ghost={models.ghost} />}
            <Camera robot={models?.robot} />
            <ViewOffset />

            {cinematic && (
                <EffectComposer multisampling={4}>
                    <N8AO aoRadius={0.12} distanceFalloff={0.6} intensity={2.2} halfRes />
                    <Bloom mipmapBlur luminanceThreshold={1.6} intensity={0.6} radius={0.65} />
                    <Vignette offset={0.25} darkness={0.7} />
                    <ToneMapping mode={ToneMappingMode.AGX} />
                </EffectComposer>
            )}
        </>
    );
}

// Procedural studio lighting: no HDR download, so it works on an offline Pi.
function Studio({ cinematic }) {
    return (
        <>
            <ambientLight intensity={0.15} />
            <directionalLight
                position={[1.0, 2.2, 1.2]} intensity={1.6}
                castShadow={!cinematic}
                shadow-mapSize={[2048, 2048]} shadow-bias={-0.0006} shadow-normalBias={0.02}
                shadow-camera-left={-0.8} shadow-camera-right={0.8}
                shadow-camera-top={0.8} shadow-camera-bottom={-0.8}
                shadow-camera-near={0.5} shadow-camera-far={5}
            />
            <Environment resolution={256} frames={1}>
                <Lightformer form="rect" intensity={3} position={[0, 4, 0]} rotation-x={Math.PI / 2} scale={[6, 6, 1]} />
                <Lightformer form="rect" intensity={1.6} position={[-4, 1.2, 1]} rotation-y={Math.PI / 2} scale={[8, 1.2, 1]} />
                <Lightformer form="rect" intensity={1.2} position={[4, 1.2, -1]} rotation-y={-Math.PI / 2} scale={[8, 1.2, 1]} />
                <Lightformer form="ring" color={ACCENT} intensity={0.6} position={[0, 1, -5]} scale={3} />
            </Environment>
        </>
    );
}

function Arm({ models }) {
    const { robot } = models;
    useFrame(() => {
        for (let i = 0; i < 6; i++) robot.joints[JOINT_NAMES[i]].setJointValue(live.q[i] * DEG);
        // Jaws are locked out of the controller's model; drive them for display only.
        const f = Math.max(0, Math.min(1, live.grip / GRIP_MAX));
        for (const n of JAW_JOINTS) {
            const j = robot.joints[n];
            if (!j) continue;
            const lo = j.limit?.lower ?? 0, hi = j.limit?.upper ?? 0;
            j.setJointValue(Math.abs(lo) > Math.abs(hi) ? lo * (1 - f) : hi * (1 - f));
        }
    }, -1);
    return (
        <>
            <primitive object={robot} />
            <primitive object={models.ghost} />
        </>
    );
}

function GhostOff({ ghost }) {
    useEffect(() => { ghost.visible = false; ghost.userData.material.uniforms.uOpacity.value = 0; }, [ghost]);
    return null;
}

// The grab point: a glowing bead on the tool tip. Clicking it is the obvious
// "move the tool" affordance; hidden while a TCP control owns the tip.
function TcpOrb({ robot, theme }) {
    const mode = useStore((s) => s.mode);
    const g = useRef();
    const ring = useRef();
    const hover = useRef(false);
    const tip = robot.links[TIP_LINK];
    const core = useMemo(() => new THREE.Color(ACCENT).multiplyScalar(4), []);

    useFrame(({ clock }, dt) => {
        tip.getWorldPosition(g.current.position);
        const s = THREE.MathUtils.damp(g.current.scale.x, hover.current ? 1.45 : 1, 18, dt);
        g.current.scale.setScalar(s);
        const t = (clock.elapsedTime % 2.4) / 2.4;
        ring.current.scale.setScalar(1 + t * 0.9);
        ring.current.material.opacity = (1 - t) * 0.55;
    });

    return (
        <group ref={g} visible={mode !== 'tcp'}>
            <mesh
                onPointerOver={(e) => { e.stopPropagation(); hover.current = true; document.body.style.cursor = 'pointer'; }}
                onPointerOut={() => { hover.current = false; document.body.style.cursor = ''; }}
                onClick={(e) => { e.stopPropagation(); useStore.setState({ mode: 'tcp' }); }}
            >
                <sphereGeometry args={[0.022, 12, 12]} />
                <meshBasicMaterial visible={false} />
            </mesh>
            <mesh>
                <sphereGeometry args={[0.0065, 24, 24]} />
                <meshBasicMaterial color={core} toneMapped={false} />
            </mesh>
            <sprite scale={0.06}>
                <spriteMaterial
                    map={haloTexture()} color={ACCENT} transparent depthWrite={false} opacity={0.55}
                    blending={theme.additive ? THREE.AdditiveBlending : THREE.NormalBlending} toneMapped={false}
                />
            </sprite>
            <Billboard>
                <mesh ref={ring}>
                    <ringGeometry args={[0.011, 0.0125, 48]} />
                    <meshBasicMaterial color={ACCENT} transparent depthWrite={false} toneMapped={false} />
                </mesh>
            </Billboard>
        </group>
    );
}

// Comet trail: points age out after LIFE seconds and fade toward the tail, so
// the line shows where the tool *just* went instead of piling up forever.
const LIFE = 6;
const MAX_PTS = 900;
function Trail({ robot, theme }) {
    const on = useStore((s) => s.S.trail);
    const nonce = useStore((s) => s.trailNonce);
    const tip = robot.links[TIP_LINK];
    const pts = useRef([]);
    const accent = useMemo(() => new THREE.Color(ACCENT), []);
    const bg = useMemo(() => new THREE.Color(theme.bg), [theme.bg]);

    const lines = useMemo(() => {
        const blending = theme.additive ? THREE.AdditiveBlending : THREE.NormalBlending;
        const common = { vertexColors: true, depthWrite: false, toneMapped: false, blending };
        return { core: makeLine({ ...common, linewidth: 2.5 }), glow: makeLine({ ...common, linewidth: 10 }) };
    }, [theme.additive]);

    useEffect(() => { pts.current = []; }, [nonce, on]);

    useFrame(({ clock, size }) => {
        const { core, glow } = lines;
        core.material.resolution.set(size.width, size.height);
        glow.material.resolution.set(size.width, size.height);
        if (!on) { core.visible = glow.visible = false; return; }
        const now = clock.elapsedTime;
        const p = pts.current;
        tip.getWorldPosition(_v);
        const last = p[p.length - 1];
        if (!last || last.v.distanceToSquared(_v) > 2.5e-7) p.push({ v: _v.clone(), t: now });
        while (p.length && (now - p[0].t > LIFE || p.length > MAX_PTS)) p.shift();
        if (p.length < 2) { core.visible = glow.visible = false; return; }

        const n = p.length;
        const pos = new Float32Array(n * 3), c1 = new Float32Array(n * 3), c2 = new Float32Array(n * 3);
        for (let i = 0; i < n; i++) {
            const a = 1 - (now - p[i].t) / LIFE;
            pos[i * 3] = p[i].v.x; pos[i * 3 + 1] = p[i].v.y; pos[i * 3 + 2] = p[i].v.z;
            if (theme.additive) {
                const k = a * a;
                c1[i * 3] = accent.r * k * 1.6; c1[i * 3 + 1] = accent.g * k * 1.6; c1[i * 3 + 2] = accent.b * k * 1.6;
                c2[i * 3] = accent.r * k * 0.18; c2[i * 3 + 1] = accent.g * k * 0.18; c2[i * 3 + 2] = accent.b * k * 0.18;
            } else {
                // No additive blending on a light background: fade into it instead.
                const k = a * a;
                for (let ch = 0; ch < 3; ch++) {
                    const A = ['r', 'g', 'b'][ch];
                    c1[i * 3 + ch] = bg[A] + (accent[A] - bg[A]) * k;
                    c2[i * 3 + ch] = bg[A] + (accent[A] - bg[A]) * k * 0.15;
                }
            }
        }
        setLine(glow, pos, c2);
        setLine(core, pos, c1);
        core.visible = glow.visible = true;
    });

    return (
        <>
            <primitive object={lines.glow} />
            <primitive object={lines.core} />
        </>
    );
}

function Camera({ robot }) {
    const ref = useRef();
    const fit = useRef(null);
    const view = useStore((s) => s.view);
    const nonce = useStore((s) => s.viewNonce);

    useEffect(() => {
        if (!robot || fit.current) return;
        // Include the Z-up parent, which hasn't rendered (and so updated) yet.
        robot.updateWorldMatrix(true, true);
        const box = new THREE.Box3().setFromObject(robot);
        const center = box.getCenter(new THREE.Vector3());
        const r = box.getSize(new THREE.Vector3()).length() * 0.5;
        fit.current = { center, dist: r / Math.sin(20 * DEG) * 1.5 };
        ref.current.minDistance = r * 0.4;
        ref.current.maxDistance = r * 12;
        go(false);
    }, [robot]); // eslint-disable-line react-hooks/exhaustive-deps

    useEffect(() => { if (fit.current) go(true); }, [nonce]); // eslint-disable-line react-hooks/exhaustive-deps

    function go(animate) {
        const { center: c, dist } = fit.current;
        const d = new THREE.Vector3(...VIEWS[view]).normalize().multiplyScalar(dist);
        ref.current.setLookAt(c.x + d.x, c.y + d.y, c.z + d.z, c.x, c.y, c.z, animate);
    }

    return <CameraControls ref={ref} makeDefault smoothTime={0.28} draggingSmoothTime={0.08} />;
}

// The side panel floats over the canvas (so the glass has something to blur),
// which would leave the arm off-centre behind it. Shift the projection instead
// of the camera, so orbiting still pivots on the arm. On narrow screens the
// panel is a bottom sheet, so the shift is vertical.
function ViewOffset() {
    const open = useStore((s) => s.prefs.panel);
    const cur = useRef(0);
    const last = useRef('');
    useFrame(({ camera, size }, dt) => {
        const wide = size.width >= 900;
        const goal = !open ? 0 : wide ? PANEL_W + 16 : Math.round(size.height * 0.48 + 8);
        cur.current = THREE.MathUtils.damp(cur.current, goal, 10, dt);
        if (Math.abs(cur.current - goal) < 0.5) cur.current = goal;
        const off = Math.round(cur.current);
        const key = `${off}:${wide}:${size.width}:${size.height}:${camera.aspect}`;
        if (key === last.current) return;
        if (off === 0) {
            camera.clearViewOffset();
            camera.aspect = size.width / size.height;
        } else if (wide) {
            camera.aspect = (size.width + off) / size.height;
            camera.setViewOffset(size.width + off, size.height, off, 0, size.width, size.height);
        } else {
            camera.aspect = size.width / (size.height + off);
            camera.setViewOffset(size.width, size.height + off, 0, off, size.width, size.height);
        }
        camera.updateProjectionMatrix();
        last.current = `${off}:${wide}:${size.width}:${size.height}:${camera.aspect}`;
    });
    return null;
}
