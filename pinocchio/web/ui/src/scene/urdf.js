import * as THREE from 'three';
import { useEffect, useState } from 'react';
import URDFLoader from 'urdf-loader';
import { useStore } from '../store.js';

export const JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
export const JAW_JOINTS = ['left_jaw_joint', 'right_jaw_joint'];
export const TIP_LINK = 'tcp_link';
export const ACCENT = '#ff6b1a';
export const AXIS_COLORS = ['#ff4d5e', '#3ddc84', '#4d8dff'];

const ceramic = new THREE.MeshPhysicalMaterial({
    color: '#e9e6e0', roughness: 0.38, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.25,
});
const graphite = new THREE.MeshPhysicalMaterial({
    color: '#2b2d31', roughness: 0.45, metalness: 0.6, clearcoat: 0.3,
});
const accent = new THREE.MeshPhysicalMaterial({
    color: ACCENT, roughness: 0.35, metalness: 0.0, clearcoat: 0.8,
});

export function linkOf(o) {
    while (o && !o.isURDFLink) o = o.parent;
    return o ? o.name : '';
}

// Hologram for the target ghost: nearly clear face-on, lit at the silhouette,
// so it says "the arm will be here" without competing with the real arm.
export function makeGhostMaterial() {
    return new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: true,     // hides the ghost's own inner faces
        toneMapped: false,
        uniforms: { uColor: { value: new THREE.Color(ACCENT) }, uOpacity: { value: 0 } },
        vertexShader: /* glsl */`
            varying vec3 vN; varying vec3 vV;
            void main() {
                vec4 mv = modelViewMatrix * vec4(position, 1.0);
                vN = normalize(normalMatrix * normal);
                vV = normalize(-mv.xyz);
                gl_Position = projectionMatrix * mv;
            }`,
        fragmentShader: /* glsl */`
            uniform vec3 uColor; uniform float uOpacity;
            varying vec3 vN; varying vec3 vV;
            void main() {
                float f = pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 2.6);
                gl_FragColor = vec4(uColor * (0.7 + 1.2 * f), (0.025 + 0.42 * f) * uOpacity);
            }`,
    });
}

/** Loads the URDF the controller runs on, a hologram clone, and an FK scratch clone. */
export function useUrdf(name) {
    const [models, setModels] = useState(null);

    useEffect(() => {
        if (!name) return;
        let cancelled = false;
        // urdf-loader resolves before its meshes arrive; wait on the manager.
        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);
        loader.parseCollision = false;
        let robot = null;
        manager.onLoad = () => {
            if (cancelled || !robot) return;
            robot.traverse((o) => {
                if (!o.isMesh) return;
                const link = linkOf(o);
                o.material = /jaw/i.test(link) ? accent
                    : /base|tool|grip/i.test(link) ? graphite : ceramic;
                o.castShadow = o.receiveShadow = true;
            });
            const ghost = robot.clone();
            const gmat = makeGhostMaterial();
            ghost.traverse((o) => {
                if (!o.isMesh) return;
                o.material = gmat;
                o.castShadow = o.receiveShadow = false;
                o.renderOrder = 2;
                // The base never moves; a ghost of it would just tint the real one.
                if (linkOf(o) === 'base_link') o.visible = false;
            });
            ghost.userData.material = gmat;
            // Never rendered: used to run FK for path previews without touching
            // the visible models.
            const scratch = robot.clone();

            // Same bound as Kinematics::computeRobotReachFromModel: sum of the
            // joint offsets from the tool back to the base.
            let maxReach = 0;
            for (let o = robot.links[TIP_LINK]; o && o !== robot; o = o.parent) {
                if (o.isURDFJoint) maxReach += o.position.length();
            }

            useStore.setState({
                limits: JOINT_NAMES.map((n) => {
                    const l = robot.joints[n].limit;
                    return { lower: (l?.lower ?? -Math.PI) * 180 / Math.PI, upper: (l?.upper ?? Math.PI) * 180 / Math.PI };
                }),
            });
            setModels({ robot, ghost, scratch, maxReach });
        };
        loader.load(`/assets/${name}`, (r) => { robot = r; });
        return () => { cancelled = true; };
    }, [name]);

    return models;
}

const DEG = Math.PI / 180;
const _p = new THREE.Vector3();

/** TCP position (URDF base frame) of `scratch` at joint angles `qDeg`. */
export function fkPos(scratch, qDeg, out = new THREE.Vector3()) {
    for (let i = 0; i < 6; i++) scratch.joints[JOINT_NAMES[i]].setJointValue(qDeg[i] * DEG);
    scratch.updateMatrixWorld(true);
    return scratch.links[TIP_LINK].getWorldPosition(out);
}

/** Sampled TCP path for a joint-interpolated move from qa to qb (degrees). */
export function jointPath(scratch, qa, qb, samples = 32, flat = []) {
    const q = new Array(6);
    for (let k = 0; k <= samples; k++) {
        const t = k / samples;
        for (let i = 0; i < 6; i++) q[i] = qa[i] * (1 - t) + qb[i] * t;
        fkPos(scratch, q, _p);
        flat.push(_p.x, _p.y, _p.z);
    }
    return flat;
}
