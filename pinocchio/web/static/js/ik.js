// Damped-least-squares IK over a urdf-loader joint chain.
//
// This exists ONLY to animate the translucent "plan" ghost while the user drags
// the TCP gizmo -- the real move is still solved by Pinocchio on the controller
// when the drag is committed. Small disagreements here are cosmetic.
import * as THREE from 'three';

const _axis = new THREE.Vector3();
const _pos = new THREE.Vector3();
const _tip = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _mat = new THREE.Matrix4();
const _rot = new THREE.Matrix4();

// Solves the 6x6 system A x = b in place by Gaussian elimination with partial
// pivoting. Returns false if A is numerically singular (damping should prevent it).
function solve6(A, b, x) {
    const n = 6;
    for (let col = 0; col < n; col++) {
        let piv = col;
        for (let r = col + 1; r < n; r++) {
            if (Math.abs(A[r * n + col]) > Math.abs(A[piv * n + col])) piv = r;
        }
        if (Math.abs(A[piv * n + col]) < 1e-12) return false;
        if (piv !== col) {
            for (let c = 0; c < n; c++) {
                const t = A[col * n + c]; A[col * n + c] = A[piv * n + c]; A[piv * n + c] = t;
            }
            const t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        const d = A[col * n + col];
        for (let r = col + 1; r < n; r++) {
            const f = A[r * n + col] / d;
            if (f === 0) continue;
            for (let c = col; c < n; c++) A[r * n + c] -= f * A[col * n + c];
            b[r] -= f * b[col];
        }
    }
    for (let r = n - 1; r >= 0; r--) {
        let s = b[r];
        for (let c = r + 1; c < n; c++) s -= A[r * n + c] * x[c];
        x[r] = s / A[r * n + r];
    }
    return true;
}

export class ChainIK {
    /**
     * @param {object} robot   URDFRobot from urdf-loader
     * @param {string[]} jointNames  actuated joints, base -> tip order
     * @param {string} tipName       link whose frame is driven to the target
     */
    constructor(robot, jointNames, tipName) {
        this.robot = robot;
        this.joints = jointNames.map(n => robot.joints[n]);
        this.tip = robot.links[tipName];
        this.n = this.joints.length;
        this.J = new Float64Array(6 * this.n);
        this.A = new Float64Array(36);
        this.b = new Float64Array(6);
        this.y = new Float64Array(6);
        this.limits = this.joints.map(j => ({
            lower: j.limit && isFinite(j.limit.lower) ? j.limit.lower : -Math.PI,
            upper: j.limit && isFinite(j.limit.upper) ? j.limit.upper : Math.PI,
        }));
    }

    // Geometric Jacobian in the world frame: for revolute joint i with world
    // axis a and origin p, Jv = a x (tip - p) and Jw = a.
    _jacobian() {
        this.robot.updateMatrixWorld(true);
        this.tip.getWorldPosition(_tip);
        for (let i = 0; i < this.n; i++) {
            const j = this.joints[i];
            j.getWorldQuaternion(_quat);
            j.getWorldPosition(_pos);
            _axis.copy(j.axis).applyQuaternion(_quat).normalize();
            const rx = _tip.x - _pos.x, ry = _tip.y - _pos.y, rz = _tip.z - _pos.z;
            this.J[0 * this.n + i] = _axis.y * rz - _axis.z * ry;
            this.J[1 * this.n + i] = _axis.z * rx - _axis.x * rz;
            this.J[2 * this.n + i] = _axis.x * ry - _axis.y * rx;
            this.J[3 * this.n + i] = _axis.x;
            this.J[4 * this.n + i] = _axis.y;
            this.J[5 * this.n + i] = _axis.z;
        }
    }

    _error(target, out) {
        this.tip.updateWorldMatrix(true, false);
        _mat.copy(this.tip.matrixWorld);
        out[0] = target.elements[12] - _mat.elements[12];
        out[1] = target.elements[13] - _mat.elements[13];
        out[2] = target.elements[14] - _mat.elements[14];
        // Orientation error as a world-frame rotation vector: axis-angle of R_t * R_c^T.
        _quat.setFromRotationMatrix(_mat).invert();
        _rot.copy(target);
        const qt = new THREE.Quaternion().setFromRotationMatrix(_rot);
        qt.multiply(_quat);
        const s = Math.sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z);
        const ang = 2 * Math.atan2(s, qt.w);
        const k = s < 1e-9 ? 0 : ang / s;
        out[3] = qt.x * k; out[4] = qt.y * k; out[5] = qt.z * k;
    }

    /**
     * @param {THREE.Matrix4} target  desired tip pose in world (URDF) coordinates
     * @param {number[]} qSeed        starting joint angles (rad)
     * @returns {{q:number[], ok:boolean, err:number}}
     */
    solve(target, qSeed, { maxIters = 60, posTol = 1e-4, rotTol = 1e-3, lambda = 0.05 } = {}) {
        const n = this.n;
        const q = qSeed.slice();
        const e = new Float64Array(6);
        let ok = false, err = Infinity;

        for (let it = 0; it < maxIters; it++) {
            for (let i = 0; i < n; i++) this.joints[i].setJointValue(q[i]);
            this._jacobian();
            this._error(target, e);

            const ep = Math.hypot(e[0], e[1], e[2]);
            const er = Math.hypot(e[3], e[4], e[5]);
            err = ep;
            if (ep < posTol && er < rotTol) { ok = true; break; }

            // A = J J^T + lambda^2 I, then dq = J^T A^-1 e
            for (let r = 0; r < 6; r++) {
                for (let c = 0; c < 6; c++) {
                    let s = 0;
                    for (let i = 0; i < n; i++) s += this.J[r * n + i] * this.J[c * n + i];
                    this.A[r * 6 + c] = s + (r === c ? lambda * lambda : 0);
                }
                this.b[r] = e[r];
            }
            if (!solve6(this.A, this.b, this.y)) break;

            let maxStep = 0;
            const dq = new Float64Array(n);
            for (let i = 0; i < n; i++) {
                let s = 0;
                for (let r = 0; r < 6; r++) s += this.J[r * n + i] * this.y[r];
                dq[i] = s;
                maxStep = Math.max(maxStep, Math.abs(s));
            }
            const scale = maxStep > 0.2 ? 0.2 / maxStep : 1;
            for (let i = 0; i < n; i++) {
                q[i] = Math.min(this.limits[i].upper,
                       Math.max(this.limits[i].lower, q[i] + dq[i] * scale));
            }
        }
        for (let i = 0; i < n; i++) this.joints[i].setJointValue(q[i]);
        return { q, ok, err };
    }
}
