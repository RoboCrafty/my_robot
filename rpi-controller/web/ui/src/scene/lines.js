import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';

export function makeLine(opts) {
    const l = new Line2(new LineGeometry(), new LineMaterial({ transparent: true, ...opts }));
    l.frustumCulled = false;
    l.visible = false;
    return l;
}

/** Replace a Line2's points (flat xyz array), optionally with per-vertex colours. */
export function setLine(line, flat, colors) {
    line.geometry.dispose();
    line.geometry = new LineGeometry();
    line.geometry.setPositions(flat);
    if (colors) line.geometry.setColors(colors);
    if (line.material.dashed) line.computeLineDistances();
}

let haloTex = null;
/** Soft radial sprite used as a cheap glow when bloom is off. */
export function haloTexture() {
    if (haloTex) return haloTex;
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const g = c.getContext('2d');
    const grd = g.createRadialGradient(64, 64, 0, 64, 64, 64);
    grd.addColorStop(0, 'rgba(255,255,255,1)');
    grd.addColorStop(0.25, 'rgba(255,255,255,0.45)');
    grd.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = grd;
    g.fillRect(0, 0, 128, 128);
    haloTex = new THREE.CanvasTexture(c);
    return haloTex;
}
