// Interactive 3D viewer for the Parol6.
//
// Everything is kept in native URDF coordinates (metres, Z-up) so poses, the TCP
// trail and the jog gizmo need no frame conversion -- only the camera's up
// vector is changed. The viewer never talks to the controller itself; it emits
// events and app.js turns them into commands.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import URDFLoader from 'urdf-loader';
import { ChainIK } from './ik.js';

export const JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
export const AXIS_COLORS = [0xe5484d, 0x37c871, 0x4c8bf5];
const TIP_LINK = 'tcp_link';
const DRAG_FULL_SCALE_PX = 120;   // pixels of drag that map to 100% jog speed

const _v = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _euler = new THREE.Euler();

// The controller speaks Pinocchio RPY (R = Rz*Ry*Rx), which is three.js's 'ZYX'
// Euler order -- NOT 'XYZ'. Getting this wrong silently rotates every pose sent.
function quatToRpy(q) {
    _euler.setFromQuaternion(q, 'ZYX');
    return [_euler.x, _euler.y, _euler.z];
}

export class RobotViewer {
    constructor(container, opts = {}) {
        THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

        this.container = container;
        this.onDragJog = opts.onDragJog || (() => {});
        this.onDragEnd = opts.onDragEnd || (() => {});
        this.onPlanChange = opts.onPlanChange || (() => {});

        this.robot = null;
        this.ghost = null;
        this.ik = null;
        this.frame = 'base';
        this.gizmoEnabled = true;
        this.planning = false;
        this._q = new Array(6).fill(0);

        this._initScene();
        this._initTrail();
        this._initHandles();
        this._initPointer();

        this._ro = new ResizeObserver(() => this._resize());
        this._ro.observe(container);
        this._resize();
        this._renderer.setAnimationLoop(() => this._tick());
    }

    // ---------------------------------------------------------------- scene
    _initScene() {
        const scene = this.scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0d1017);

        this.camera = new THREE.PerspectiveCamera(45, 1, 0.02, 50);
        this.camera.position.set(0.65, -0.65, 0.55);

        this._renderer = new THREE.WebGLRenderer({ antialias: true });
        this._renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
        this._renderer.outputColorSpace = THREE.SRGBColorSpace;
        this._renderer.shadowMap.enabled = true;
        this._renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this._renderer.domElement);

        const orbit = this.orbit = new OrbitControls(this.camera, this._renderer.domElement);
        orbit.enableDamping = true;
        orbit.dampingFactor = 0.08;
        orbit.target.set(0, 0, 0.25);
        orbit.minDistance = 0.2;
        orbit.maxDistance = 4;

        scene.add(new THREE.HemisphereLight(0x9fb4d0, 0x181c24, 1.4));
        const key = new THREE.DirectionalLight(0xffffff, 2.0);
        key.position.set(0.8, -1.0, 1.6);
        key.castShadow = true;
        key.shadow.mapSize.set(2048, 2048);
        key.shadow.camera.near = 0.2;
        key.shadow.camera.far = 5;
        const d = 0.8;
        Object.assign(key.shadow.camera, { left: -d, right: d, top: d, bottom: -d });
        key.shadow.bias = -0.0008;
        scene.add(key);
        const fill = new THREE.DirectionalLight(0x8fa8c8, 0.7);
        fill.position.set(-1.2, 0.9, 0.6);
        scene.add(fill);

        const floor = new THREE.Mesh(
            new THREE.PlaneGeometry(6, 6),
            new THREE.ShadowMaterial({ opacity: 0.35 }));
        floor.receiveShadow = true;
        scene.add(floor);

        const grid = new THREE.GridHelper(2, 40, 0x35415a, 0x1d2531);
        grid.rotation.x = Math.PI / 2;   // GridHelper is XZ; we are Z-up
        grid.material.transparent = true;
        grid.material.opacity = 0.65;
        scene.add(grid);
        scene.add(this._makeTriad(0.12, 0.0025));   // world frame at the origin
    }

    _makeTriad(len, rad) {
        const g = new THREE.Group();
        for (let a = 0; a < 3; a++) {
            const m = new THREE.MeshBasicMaterial({ color: AXIS_COLORS[a] });
            const shaft = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, len, 8), m);
            shaft.position.setComponent(a, len / 2);
            shaft.rotation.set(a === 1 ? 0 : Math.PI / 2, 0, a === 0 ? -Math.PI / 2 : 0);
            g.add(shaft);
        }
        return g;
    }

    // ---------------------------------------------------------------- model
    async load(urdfUrl) {
        // urdf-loader's own callback fires as soon as the XML is parsed, while the
        // STL/DAE meshes are still in flight -- wait on the LoadingManager instead,
        // or the material override and camera fit would run on an empty robot.
        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);
        loader.parseCollision = false;
        const robot = await new Promise((res, rej) => {
            let model = null;
            manager.onLoad = () => res(model);
            manager.onError = (url) => rej(new Error(`could not load ${url}`));
            loader.load(urdfUrl, r => { model = r; }, null, rej);
        });

        robot.traverse(o => {
            if (!o.isMesh) return;
            o.castShadow = true;
            o.receiveShadow = true;
            const src = Array.isArray(o.material) ? o.material[0] : o.material;
            const c = src && src.color ? src.color.clone() : new THREE.Color(0x9aa5b4);
            // The gripper DAE ships an almost-black diffuse colour, which reads as a
            // hole in the arm against a dark scene.
            if (c.r + c.g + c.b < 0.15) c.setHex(0x9aa5b4);
            o.material = new THREE.MeshStandardMaterial({
                color: c, metalness: 0.45, roughness: 0.42,
            });
        });

        this.robot = robot;
        this.scene.add(robot);
        this.ik = new ChainIK(robot, JOINT_NAMES, TIP_LINK);
        this.tip = robot.links[TIP_LINK];

        this.ghost = robot.clone();
        this.ghost.traverse(o => {
            if (!o.isMesh) return;
            o.castShadow = false;
            o.receiveShadow = false;
            o.material = new THREE.MeshStandardMaterial({
                color: 0x4c8bf5, transparent: true, opacity: 0.22,
                depthWrite: false, metalness: 0.1, roughness: 0.8,
            });
        });
        this.ghost.visible = false;
        this.scene.add(this.ghost);
        this.ghostIK = new ChainIK(this.ghost, JOINT_NAMES, TIP_LINK);

        this._initGizmo();
        this.setJoints(new Array(6).fill(0));
        this._fitCamera();
        return robot;
    }

    // Derive the orbit target and view distance from the model itself, so the
    // presets stay correct if the URDF or tool changes.
    _fitCamera() {
        const box = new THREE.Box3().setFromObject(this.robot);
        const size = box.getSize(new THREE.Vector3());
        box.getCenter(this._focus = new THREE.Vector3());
        const radius = Math.max(size.x, size.y, size.z) * 0.5;
        this._fitDist = radius / Math.sin(THREE.MathUtils.degToRad(this.camera.fov) * 0.5) * 1.45;
        this.orbit.target.copy(this._focus);
        this.orbit.minDistance = radius * 0.4;
        this.orbit.maxDistance = radius * 20;
        this.setView('iso');
    }

    jointLimits() {
        if (!this.robot) return null;
        return JOINT_NAMES.map(n => {
            const l = this.robot.joints[n].limit;
            return { lower: (l?.lower ?? -Math.PI) * 180 / Math.PI, upper: (l?.upper ?? Math.PI) * 180 / Math.PI };
        });
    }

    /** @param {number[]} deg live joint feedback, degrees */
    setJoints(deg) {
        if (!this.robot) return;
        for (let i = 0; i < 6; i++) {
            this._q[i] = deg[i] * Math.PI / 180;
            this.robot.joints[JOINT_NAMES[i]].setJointValue(this._q[i]);
        }
        this.robot.updateMatrixWorld(true);
        this._syncHandles();
        this._pushTrail();
    }

    /** TCP pose in world/URDF coords: {pos:[x,y,z], rpy:[rx,ry,rz]} */
    tcpPose() {
        if (!this.tip) return null;
        this.tip.getWorldPosition(_v);
        this.tip.getWorldQuaternion(_q);
        return { pos: [_v.x, _v.y, _v.z], rpy: quatToRpy(_q) };
    }

    // ---------------------------------------------------------------- trail
    _initTrail() {
        this.trailEnabled = true;
        this._trailPts = [];
        this._trailMax = 4000;
        this._trailMat = new LineMaterial({
            color: 0x4c8bf5, linewidth: 2.5, transparent: true, opacity: 0.9, dashed: false,
        });
        this._trail = new Line2(new LineGeometry(), this._trailMat);
        this._trail.frustumCulled = false;
        this._trail.visible = false;
        this.scene.add(this._trail);
    }

    _pushTrail() {
        if (!this.trailEnabled || !this.tip) return;
        this.tip.getWorldPosition(_v);
        const p = this._trailPts;
        const n = p.length;
        if (n >= 3) {
            const dx = _v.x - p[n - 3], dy = _v.y - p[n - 2], dz = _v.z - p[n - 1];
            if (dx * dx + dy * dy + dz * dz < 1e-6) return;   // <1 mm: not worth a vertex
        }
        p.push(_v.x, _v.y, _v.z);
        if (p.length > this._trailMax * 3) p.splice(0, p.length - this._trailMax * 3);
        if (p.length >= 6) {
            this._trail.geometry.dispose();
            this._trail.geometry = new LineGeometry();
            this._trail.geometry.setPositions(p);
            this._trail.computeLineDistances();
            this._trail.visible = true;
        }
    }

    setTrail(on) {
        this.trailEnabled = on;
        if (!on) this._trail.visible = false;
        else if (this._trailPts.length >= 6) this._trail.visible = true;
    }

    clearTrail() {
        this._trailPts.length = 0;
        this._trail.visible = false;
    }

    // ------------------------------------------------------- TCP jog handles
    // One draggable arrow per translation axis and one ring per rotation axis,
    // parented to a group that follows the TCP. Dragging emits a normalised
    // -1..1 rate; app.js maps that onto `cartjogvel`.
    _initHandles() {
        const g = this.handles = new THREE.Group();
        g.renderOrder = 10;
        this.scene.add(g);
        this._handleMeshes = [];

        const L = 0.062, R = 0.045;
        for (let a = 0; a < 3; a++) {
            const mat = new THREE.MeshBasicMaterial({
                color: AXIS_COLORS[a], depthTest: false, transparent: true, opacity: 0.95,
            });
            const arrow = new THREE.Group();
            const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0022, 2 * L, 10), mat);
            arrow.add(shaft);
            for (const s of [-1, 1]) {
                const cone = new THREE.Mesh(new THREE.ConeGeometry(0.0085, 0.022, 14), mat);
                cone.position.y = s * L;
                cone.rotation.x = s > 0 ? 0 : Math.PI;
                arrow.add(cone);
            }
            // A fat invisible cylinder makes the thin arrow easy to grab.
            const pick = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 2 * L + 0.04, 8),
                new THREE.MeshBasicMaterial({ visible: false }));
            arrow.add(pick);
            arrow.rotation.set(a === 1 ? 0 : Math.PI / 2, 0, a === 0 ? -Math.PI / 2 : 0);
            arrow.traverse(o => { o.renderOrder = 10; o.userData.axis = a; });
            arrow.userData = { axis: a, mat, kind: 'lin' };
            g.add(arrow);
            this._handleMeshes.push(arrow);

            const rmat = new THREE.MeshBasicMaterial({
                color: AXIS_COLORS[a], depthTest: false, transparent: true, opacity: 0.55,
            });
            const ring = new THREE.Mesh(new THREE.TorusGeometry(R, 0.0028, 8, 96), rmat);
            const pickRing = new THREE.Mesh(new THREE.TorusGeometry(R, 0.012, 6, 48),
                new THREE.MeshBasicMaterial({ visible: false }));
            ring.add(pickRing);
            if (a === 0) ring.rotation.y = Math.PI / 2;
            else if (a === 1) ring.rotation.x = Math.PI / 2;
            ring.traverse(o => { o.renderOrder = 10; o.userData.axis = 3 + a; });
            ring.userData = { axis: 3 + a, mat: rmat, kind: 'rot' };
            g.add(ring);
            this._handleMeshes.push(ring);
        }
    }

    _syncHandles() {
        if (!this.tip) return;
        this.tip.getWorldPosition(_v);
        this.handles.position.copy(_v);
        if (this.frame === 'tool') {
            this.tip.getWorldQuaternion(_q);
            this.handles.quaternion.copy(_q);
        } else {
            this.handles.quaternion.identity();
        }
        this.handles.updateMatrixWorld(true);
    }

    setFrame(frame) { this.frame = frame; this._syncHandles(); }
    setGizmoVisible(on) { this.gizmoEnabled = on; this.handles.visible = on && !this.planning; }

    /** Pulse an axis handle so a hovered jog button shows what it will move. */
    highlightAxis(axis) {
        for (const h of this._handleMeshes) {
            const on = h.userData.axis === axis;
            const base = h.userData.kind === 'rot' ? 0.55 : 0.95;
            h.userData.mat.opacity = axis == null ? base : (on ? 1.0 : base * 0.25);
            h.scale.setScalar(on ? 1.12 : 1.0);
        }
    }

    // ------------------------------------------------------------- pointer
    _initPointer() {
        const dom = this._renderer.domElement;
        this._ray = new THREE.Raycaster();
        this._drag = null;
        this._hover = null;

        const ndc = (e) => {
            const r = dom.getBoundingClientRect();
            return new THREE.Vector2(
                ((e.clientX - r.left) / r.width) * 2 - 1,
                -((e.clientY - r.top) / r.height) * 2 + 1);
        };
        const pick = (e) => {
            if (!this.handles.visible) return null;
            this._ray.setFromCamera(ndc(e), this.camera);
            const hits = this._ray.intersectObjects(this._handleMeshes, true);
            if (!hits.length) return null;
            let o = hits[0].object;
            while (o && o.userData.kind === undefined) o = o.parent;
            return o ? { handle: o, point: hits[0].point } : null;
        };

        dom.addEventListener('pointermove', (e) => {
            if (this._drag) { this._onDragMove(e, dom); return; }
            const h = pick(e);
            const axis = h ? h.handle.userData.axis : null;
            if (axis !== this._hover) { this._hover = axis; this.highlightAxis(axis); }
            dom.style.cursor = h ? 'grab' : '';
        });

        dom.addEventListener('pointerdown', (e) => {
            const h = pick(e);
            if (!h) return;
            e.preventDefault();
            dom.setPointerCapture(e.pointerId);
            this.orbit.enabled = false;
            dom.style.cursor = 'grabbing';

            const u = h.handle.userData;
            // Screen-space direction the handle moves in: the axis itself for a
            // translation arrow, the tangent at the grab point for a ring.
            const origin = this.handles.getWorldPosition(new THREE.Vector3());
            const axisDir = new THREE.Vector3().setFromMatrixColumn(this.handles.matrixWorld, u.axis % 3).normalize();
            let dir3;
            if (u.kind === 'lin') {
                dir3 = axisDir;
            } else {
                dir3 = new THREE.Vector3().subVectors(h.point, origin).cross(axisDir).normalize().negate();
            }
            const a = origin.clone().project(this.camera);
            const b = origin.clone().addScaledVector(dir3, 0.05).project(this.camera);
            const screen = new THREE.Vector2(b.x - a.x, -(b.y - a.y));
            if (screen.lengthSq() < 1e-8) screen.set(1, 0);
            screen.normalize();

            this._drag = { axis: u.axis, screen, x0: e.clientX, y0: e.clientY, last: 0, pointerId: e.pointerId };
            this.highlightAxis(u.axis);
        });

        const end = (e) => {
            if (!this._drag) return;
            if (dom.hasPointerCapture(this._drag.pointerId)) dom.releasePointerCapture(this._drag.pointerId);
            const axis = this._drag.axis;
            this._drag = null;
            this.orbit.enabled = true;
            dom.style.cursor = '';
            this.highlightAxis(null);
            this.onDragEnd(axis);
        };
        dom.addEventListener('pointerup', end);
        dom.addEventListener('pointercancel', end);
        window.addEventListener('blur', end);
    }

    _onDragMove(e, dom) {
        const d = this._drag;
        const px = (e.clientX - d.x0) * d.screen.x + (e.clientY - d.y0) * d.screen.y;
        const rate = Math.max(-1, Math.min(1, px / DRAG_FULL_SCALE_PX));
        if (Math.abs(rate - d.last) < 0.02) return;
        d.last = rate;
        this.onDragJog(d.axis, rate);
    }

    // ------------------------------------------------------- plan / ghost
    _initGizmo() {
        const tc = this.gizmo = new TransformControls(this.camera, this._renderer.domElement);
        tc.setSize(0.9);
        tc.setSpace('world');
        this.planTarget = new THREE.Object3D();
        this.scene.add(this.planTarget);
        tc.attach(this.planTarget);
        tc.visible = false;
        tc.enabled = false;
        this.scene.add(tc);

        tc.addEventListener('dragging-changed', (ev) => { this.orbit.enabled = !ev.value; });
        tc.addEventListener('objectChange', () => this._solveGhost());
    }

    setPlanning(on) {
        if (!this.gizmo) return false;
        this.planning = on;
        this.gizmo.visible = on;
        this.gizmo.enabled = on;
        this.ghost.visible = on;
        this.handles.visible = !on && this.gizmoEnabled;
        if (on) {
            this.tip.getWorldPosition(_v);
            this.tip.getWorldQuaternion(_q);
            this.planTarget.position.copy(_v);
            this.planTarget.quaternion.copy(_q);
            this.planTarget.updateMatrixWorld(true);
            this._ghostSeed = this._q.slice();
            this._solveGhost();
        }
        return true;
    }

    setPlanMode(mode) { this.gizmo.setMode(mode); }        // 'translate' | 'rotate'
    setPlanSpace(space) { this.gizmo.setSpace(space); }    // 'world' | 'local'

    _solveGhost() {
        if (!this.planning) return;
        this.planTarget.updateMatrixWorld(true);
        const r = this.ghostIK.solve(this.planTarget.matrixWorld, this._ghostSeed);
        if (r.ok) this._ghostSeed = r.q;
        this.ghost.updateMatrixWorld(true);
        this.ghost.traverse(o => { if (o.isMesh) o.material.color.setHex(r.ok ? 0x4c8bf5 : 0xe5484d); });
        this.onPlanChange({
            pos: this.planTarget.position.toArray(),
            rpy: quatToRpy(this.planTarget.quaternion),
            reachable: r.ok,
            q: r.q.map(v => v * 180 / Math.PI),
        });
    }

    /** Snap the plan gizmo back onto the live TCP. */
    resetPlan() { if (this.planning) this.setPlanning(true); }

    // ---------------------------------------------------------------- view
    setView(name) {
        const t = this._focus || this.orbit.target;
        const d = this._fitDist || 1.0;
        const dir = {
            iso: [0.62, -0.68, 0.42], front: [0, -1, 0.16], side: [1, 0, 0.16], top: [0.001, -0.02, 1],
        }[name];
        if (!dir) return;
        const v = new THREE.Vector3(...dir).normalize().multiplyScalar(d);
        this.orbit.target.copy(t);
        this.camera.position.copy(t).add(v);
        this.orbit.update();
    }

    _resize() {
        const w = this.container.clientWidth, h = this.container.clientHeight;
        if (!w || !h) return;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this._renderer.setSize(w, h, false);
        this._trailMat.resolution.set(w, h);
    }

    _tick() {
        this.orbit.update();
        this._renderer.render(this.scene, this.camera);
    }

    dispose() {
        this._ro.disconnect();
        this._renderer.setAnimationLoop(null);
        this._renderer.dispose();
    }
}
