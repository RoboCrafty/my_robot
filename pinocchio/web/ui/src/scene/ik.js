// Damped-least-squares IK over a urdf-loader joint chain.
//
// The defaults mirror Kinematics::InverseKinematics_Positional in
// pinocchio/src/kinematics.hpp (tolerances, damping, step clamp, iteration cap,
// reach check). When the preview is solved the same way from the same seed as
// the controller, "reachable" here means the controller will actually move.
import * as THREE from 'three';

export const CONTROLLER_IK = {
    posTol: 1e-3, rotTol: 1e-2, maxIters: 100, damping: 1e-4, maxStep: 0.5,
};

const _axis = new THREE.Vector3();
const _pos = new THREE.Vector3();
const _tip = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _qt = new THREE.Quaternion();

// Solves the 6x6 system A x = b in place by Gaussian elimination with partial
// pivoting. Returns false if A is numerically singular (damping should prevent it).
function solve6(A, b, x) {
    const n = 6;
    for (let col = 0; col < n; col++) {
        let piv = col;
        for (let r = col + 1; r < n; r++) {
            if (Math.abs(A[r * n + col]) > Math.abs(A[piv * n + col])) piv = r;
        }
        if (Math.abs(A[piv * n + col]) < 1e-14) return false;
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

// Smallest eigenvalue of a symmetric 6x6 matrix (cyclic Jacobi).
function minEig6(M) {
    const a = Float64Array.from(M);
    for (let sweep = 0; sweep < 30; sweep++) {
        let off = 0;
        for (let p = 0; p < 6; p++) for (let q = p + 1; q < 6; q++) off += a[p * 6 + q] ** 2;
        if (off < 1e-20) break;
        for (let p = 0; p < 6; p++) {
            for (let q = p + 1; q < 6; q++) {
                const apq = a[p * 6 + q];
                if (Math.abs(apq) < 1e-15) continue;
                const theta = (a[q * 6 + q] - a[p * 6 + p]) / (2 * apq);
                const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
                const c = 1 / Math.sqrt(t * t + 1), s = t * c;
                for (let k = 0; k < 6; k++) {
                    const akp = a[k * 6 + p], akq = a[k * 6 + q];
                    a[k * 6 + p] = c * akp - s * akq;
                    a[k * 6 + q] = s * akp + c * akq;
                }
                for (let k = 0; k < 6; k++) {
                    const apk = a[p * 6 + k], aqk = a[q * 6 + k];
                    a[p * 6 + k] = c * apk - s * aqk;
                    a[q * 6 + k] = s * apk + c * aqk;
                }
            }
        }
    }
    let m = Infinity;
    for (let i = 0; i < 6; i++) m = Math.min(m, a[i * 7]);
    return m;
}

export class ChainIK {
    /**
     * @param {object} robot   URDFRobot from urdf-loader
     * @param {string[]} jointNames  actuated joints, base -> tip order
     * @param {string} tipName       link whose frame is driven to the target
     */
    constructor(robot, jointNames, tipName) {
        this.robot = robot;
        this.joints = jointNames.map((n) => robot.joints[n]);
        this.tip = robot.links[tipName];
        this.n = this.joints.length;
        this.J = new Float64Array(6 * this.n);
        this.A = new Float64Array(36);
        this.b = new Float64Array(6);
        this.y = new Float64Array(6);
        this.limits = this.joints.map((j) => ({
            lower: j.limit && isFinite(j.limit.lower) ? j.limit.lower : -Math.PI,
            upper: j.limit && isFinite(j.limit.upper) ? j.limit.upper : Math.PI,
        }));
    }

    setQ(q) {
        for (let i = 0; i < this.n; i++) this.joints[i].setJointValue(q[i]);
        this.robot.updateMatrixWorld(true);
    }

    // World-aligned geometric Jacobian: for revolute joint i with world axis a
    // and origin p, Jv = a x (tip - p) and Jw = a.
    _jacobian() {
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
        const m = this.tip.matrixWorld.elements, t = target.elements;
        out[0] = t[12] - m[12];
        out[1] = t[13] - m[13];
        out[2] = t[14] - m[14];
        // Orientation error as a world-frame rotation vector: axis-angle of R_t * R_c^T.
        _quat.setFromRotationMatrix(this.tip.matrixWorld).invert();
        _qt.setFromRotationMatrix(target).multiply(_quat);
        if (_qt.w < 0) { _qt.x = -_qt.x; _qt.y = -_qt.y; _qt.z = -_qt.z; _qt.w = -_qt.w; }
        const s = Math.sqrt(_qt.x * _qt.x + _qt.y * _qt.y + _qt.z * _qt.z);
        const ang = 2 * Math.atan2(s, _qt.w);
        const k = s < 1e-9 ? 0 : ang / s;
        out[3] = _qt.x * k; out[4] = _qt.y * k; out[5] = _qt.z * k;
    }

    /** Smallest singular value of the Jacobian at the chain's current pose. */
    sigmaMin() {
        this._jacobian();
        const n = this.n, A = this.A;
        for (let r = 0; r < 6; r++) {
            for (let c = 0; c < 6; c++) {
                let s = 0;
                for (let i = 0; i < n; i++) s += this.J[r * n + i] * this.J[c * n + i];
                A[r * 6 + c] = s;
            }
        }
        return Math.sqrt(Math.max(0, minEig6(A)));
    }

    /**
     * Fraction of a world-aligned twist the arm cannot produce at its current
     * pose, computed exactly like Kinematics::resolvedRate's track_err (same
     * adaptive damping). The controller aborts `movel` when this exceeds 0.1.
     */
    trackErr(twist) {
        const sigma = this.sigmaMin();
        let lambda = CONTROLLER_IK.damping;
        if (sigma < 0.02) { const k = 1 - sigma / 0.02; lambda += 0.05 * 0.05 * k * k; }
        this._jacobian();
        const n = this.n;
        for (let r = 0; r < 6; r++) {
            for (let c = 0; c < 6; c++) {
                let s = 0;
                for (let i = 0; i < n; i++) s += this.J[r * n + i] * this.J[c * n + i];
                this.A[r * 6 + c] = s + (r === c ? lambda : 0);
            }
            this.b[r] = twist[r];
        }
        if (!solve6(this.A, this.b, this.y)) return { sigma, trackErr: 1 };
        // With y = (JJ^T + lambda I)^-1 t, the residual J J^T y - t is -lambda y.
        const tn = Math.hypot(...twist);
        return { sigma, trackErr: tn > 1e-12 ? lambda * Math.hypot(...this.y) / tn : 0 };
    }

    /**
     * @param {THREE.Matrix4} target  desired tip pose in world coordinates
     * @param {number[]} qSeed        starting joint angles (rad)
     * @param {object} [o]            overrides of CONTROLLER_IK, plus
     *                                `reachPos` (target position in the base frame)
     *                                and `maxReach` for the controller's reach check
     * @returns {{q:number[], ok:boolean, err:number, iters:number}}
     */
    solve(target, qSeed, o = {}) {
        const { posTol, rotTol, maxIters, damping, maxStep } = { ...CONTROLLER_IK, ...o };
        const n = this.n;
        const q = qSeed.slice();
        if (o.maxReach && o.reachPos && Math.hypot(...o.reachPos) > o.maxReach) {
            return { q, ok: false, err: Infinity, iters: 0 };
        }
        const e = new Float64Array(6);
        const dq = new Float64Array(n);
        let err = Infinity;

        for (let it = 0; it < maxIters; it++) {
            this.setQ(q);
            this._error(target, e);
            const ep = Math.hypot(e[0], e[1], e[2]);
            const er = Math.hypot(e[3], e[4], e[5]);
            err = ep;
            if (ep < posTol && er < rotTol) return { q, ok: true, err, iters: it };

            // dq = J^T (J J^T + damping I)^-1 e
            this._jacobian();
            for (let r = 0; r < 6; r++) {
                for (let c = 0; c < 6; c++) {
                    let s = 0;
                    for (let i = 0; i < n; i++) s += this.J[r * n + i] * this.J[c * n + i];
                    this.A[r * 6 + c] = s + (r === c ? damping : 0);
                }
                this.b[r] = e[r];
            }
            if (!solve6(this.A, this.b, this.y)) break;
            let norm = 0;
            for (let i = 0; i < n; i++) {
                let s = 0;
                for (let r = 0; r < 6; r++) s += this.J[r * n + i] * this.y[r];
                dq[i] = s;
                norm += s * s;
            }
            norm = Math.sqrt(norm);
            const scale = norm > maxStep ? maxStep / norm : 1;
            for (let i = 0; i < n; i++) {
                q[i] = Math.min(this.limits[i].upper, Math.max(this.limits[i].lower, q[i] + dq[i] * scale));
            }
        }
        this.setQ(q);
        return { q, ok: false, err, iters: maxIters };
    }
}
