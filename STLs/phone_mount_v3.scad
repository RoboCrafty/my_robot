// ============================================================
//  Modular phone / camera mount v3
//  table base (clamp or glue) + snap-fit rod + top part:
//     top_part = "cap"     -> 1/4" camera-screw cap (phone ball head)
//     top_part = "cam_arm" -> L-shaped arm, top-down 34x34 camera
//  part = print_all | assembly | base | rod | cap | cam_arm
// ============================================================
part     = "print_all";
top_part = "cam_arm";      // "cap" or "cam_arm"  (what goes on top of the rod)

TARGET_H = 800;            // table surface -> top of the top part
N        = 4;

base_mode = "clamp";       // "clamp" or "glue"
table_t   = 20;
plate_x   = 50;
plate_y   = 100;
plate_t   = 5;
PED_H     = 20;
hook_wall = 8;
arm_len   = 45;            // clamp bottom arm length
clamp_bolt = 6.6;
nut_af    = 10.2;
nut_h     = 5.6;

OUT  = 30;
WALL = 3;
IN   = OUT - 2*WALL;
CLR  = 0.2;
SP   = IN - 2*CLR;
SPW  = 2.4;
SPI  = SP - 2*SPW;
SP_LEN   = 40;
SHOULDER = 4;

TW   = 10;
SL   = 1.2;
TL   = 28;
BUMP = 1.2;

R_OUT  = 3;
R_EDGE = 1.2;
CH     = 0.6;              // lead-in chamfer on spigot tip / socket mouth

// ---------- 1/4" cap ----------
top_mode  = "bolt";        // "bolt" or "nut"
CAP_SOCK  = 42;            // socket depth (2 mm more than SP_LEN so the spigot never bottoms out)
CAP_PLATE = 10;
UNC_D     = 6.6;
UNC_AF    = 11.5;
HEX_DEPTH = 5.0;

// ---------- camera arm (top-down camera, e.g. 34x34 USB module) ----------
CAM_SIZE     = 34;         // board size (square)
CAM_POCKET   = CAM_SIZE + 0.6;
CAM_POCKET_D = 5;          // pocket depth: board + a bit (open at the top)
ARM_T        = 9;          // arm thickness -> floor under camera = ARM_T - CAM_POCKET_D = 4
CAM_ARM_LEN  = 150;        // rod axis -> camera centre (horizontal reach)
PAD          = 44;         // square pad at the end of the arm
LENS_D       = 18;         // hole in the floor for the lens to poke down through
CAM_HOLES    = true;       // 4 mounting holes through the floor
CAM_HOLE_SP  = 28;         // hole spacing 28 x 28 (centre to centre)
CAM_HOLE_D   = 2.2;
CAM_CABLE    = true;       // cable notch through the +X wall of the pocket

BASE_H  = plate_t + PED_H;
CAP_H   = CAP_SOCK + CAP_PLATE;
CAM_H   = CAP_SOCK + ARM_T;
TOP_H   = (top_part == "cam_arm") ? CAM_H : CAP_H;
SEG_LEN = (TARGET_H - BASE_H - TOP_H) / N;

$fn = 48;
e = 0.01;

module rsq2d(s, r) { offset(r = r) square(s - 2*r, center = true); }
module rsq2d_rect(x, y, r) { offset(r = r) square([x - 2*r, y - 2*r], center = true); }
module sq(s, h) { linear_extrude(h) rsq2d(s, R_OUT); }

module body(s, h) {
    hull() {
        translate([0, 0, R_EDGE]) linear_extrude(h - 2*R_EDGE) rsq2d(s, R_OUT);
        linear_extrude(h) rsq2d(s - 2*R_EDGE, max(R_OUT - R_EDGE, 0.5));
    }
}

module sq_in(s, h) { translate([-s/2, -s/2, 0]) cube([s, s, h]); }

// ---------- snap bump ----------
module bump() {
    translate([-TW/2, 0, 0]) rotate([90, 0, 90]) linear_extrude(TW)
        polygon([[SP/2 - e, TL - 8], [SP/2 + BUMP, TL - 6],
                 [SP/2 + BUMP, TL - 4.5], [SP/2 - e, TL - 0.5]]);
}

// ---------- tongue slots (round ends at the root, cross slot frees the tip) ----------
module tongue_slots() {
    y0 = SP/2 - SPW - 0.5;
    dy = SPW + 1;
    for (sx = [-1, 1])
        hull() {
            translate([sx*(TW/2 + SL/2), y0, TL + SL/2]) rotate([-90,0,0]) cylinder(d = SL, h = dy);
            translate([sx*(TW/2 + SL/2), y0, 1.5])       rotate([-90,0,0]) cylinder(d = SL, h = dy);
        }
    translate([-(TW/2 + SL), y0, TL]) cube([TW + 2*SL, dy, SL]);
}

module spigot() {
    difference() {
        union() {
            difference() {
                hull() {
                    linear_extrude(SP_LEN - CH) rsq2d(SP, 2);
                    linear_extrude(SP_LEN)      rsq2d(SP - 2*CH, 1.5);
                }
                translate([0, 0, -e]) sq_in(SPI, SP_LEN + 2*e);
            }
            bump();
            mirror([0, 1, 0]) bump();
        }
        tongue_slots();
        mirror([0, 1, 0]) tongue_slots();
    }
}

module socket_windows() {
    translate([-(TW + 2)/2, -OUT/2 - 1, TL - 9]) cube([TW + 2, OUT + 2, 9.5]);
}

module socket_mouth_chamfer() {
    translate([0, 0, -e]) hull() {
        linear_extrude(e) rsq2d(IN + 2*CH, 1.5);
        translate([0, 0, CH]) linear_extrude(e) rsq2d(IN, 1.5);
    }
}

// ---------- rod segment ----------
module rod_segment() {
    difference() {
        union() {
            body(OUT, SEG_LEN);
            translate([0, 0, SEG_LEN - e]) spigot();
        }
        translate([0, 0, -1]) sq_in(IN, SEG_LEN - SHOULDER + 1);
        translate([0, 0, SEG_LEN - SHOULDER - e]) sq_in(SPI, SHOULDER + 2*e);
        socket_mouth_chamfer();
        socket_windows();
    }
}

// ---------- base ----------
module gussets() {
    for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0])
        translate([-2.5, 0, 0]) rotate([90, 0, 90]) linear_extrude(5)
            polygon([[OUT/2 - 1, plate_t - e], [OUT/2 + 28, plate_t - e], [OUT/2 - 1, BASE_H - 2]]);
    for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0, 0])
        translate([0, 2.5, 0]) rotate([90, 0, 0]) linear_extrude(5)
            polygon([[OUT/2 - 1, plate_t - e], [OUT/2 + 9, plate_t - e], [OUT/2 - 1, BASE_H - 2]]);
}

module ped_fillet() {
    hull() {
        linear_extrude(plate_t + e) rsq2d(OUT + 2*8, R_OUT + 8);
        linear_extrude(plate_t + 8) rsq2d(OUT, R_OUT);
    }
}

module clamp_hook() {
    gap = table_t + 2;
    z0  = -(gap + hook_wall);
    y0  = -plate_y/2 - hook_wall;
    difference() {
        union() {
            translate([-plate_x/2, y0, z0]) cube([plate_x, hook_wall, gap + hook_wall + plate_t]);
            translate([-plate_x/2, y0, z0]) cube([plate_x, hook_wall + arm_len, hook_wall]);
            // tiny inside-corner fillet (kept small so the table still fits)
            translate([-plate_x/2, y0 + hook_wall, z0 + hook_wall]) rotate([90, 0, 90])
                linear_extrude(plate_x) polygon([[0, 0], [1.5, 0], [0, 1.5]]);
        }
        by = -plate_y/2 + arm_len*0.55;
        translate([0, by, z0 - 1]) cylinder(d = clamp_bolt, h = hook_wall + 2);
        translate([0, by, z0 + hook_wall - nut_h]) cylinder(d = nut_af / cos(30), h = nut_h + e, $fn = 6);
    }
}

module base() {
    union() {
        linear_extrude(plate_t) rsq2d_rect(plate_x, plate_y, 4);
        ped_fillet();
        sq(OUT, BASE_H);
        gussets();
        translate([0, 0, BASE_H - e]) spigot();
        if (base_mode == "clamp") clamp_hook();
    }
}

// ---------- top cap (1/4" camera screw) ----------
module top_cap() {
    difference() {
        body(OUT, CAP_H);
        translate([0, 0, -1]) sq_in(IN, CAP_SOCK + 1);
        socket_mouth_chamfer();
        translate([0, 0, CAP_SOCK - 1]) cylinder(d = UNC_D, h = CAP_PLATE + 2);
        if (top_mode == "bolt")
            translate([0, 0, CAP_SOCK - e]) cylinder(d = UNC_AF / cos(30), h = HEX_DEPTH, $fn = 6);
        else
            translate([0, 0, CAP_H - HEX_DEPTH]) cylinder(d = UNC_AF / cos(30), h = HEX_DEPTH + 1, $fn = 6);
        socket_windows();
    }
}

// ---------- camera arm: socket block + horizontal arm + pad with pocket ----------
// Camera sits in the pocket on TOP of the pad; its lens pokes down through the hole.
module cam_arm() {
    top_z = CAP_SOCK + ARM_T;
    difference() {
        union() {
            body(OUT, top_z);                                    // socket block
            hull() {                                             // arm, widening into the pad
                translate([0, 0, CAP_SOCK]) linear_extrude(ARM_T) rsq2d(OUT, R_OUT);
                translate([CAM_ARM_LEN, 0, CAP_SOCK]) linear_extrude(ARM_T) rsq2d(PAD, 4);
            }
            // gusset under the arm (extruded along Y)
            translate([0, 3, 0]) rotate([90, 0, 0]) linear_extrude(6)
                polygon([[OUT/2 - 1, CAP_SOCK - 22], [OUT/2 - 1, CAP_SOCK + e],
                         [OUT/2 + 40, CAP_SOCK + e]]);
        }
        // socket
        translate([0, 0, -1]) sq_in(IN, CAP_SOCK + 1);
        socket_mouth_chamfer();
        socket_windows();
        // camera pocket (open at top)
        translate([CAM_ARM_LEN - CAM_POCKET/2, -CAM_POCKET/2, top_z - CAM_POCKET_D])
            cube([CAM_POCKET, CAM_POCKET, CAM_POCKET_D + 1]);
        // lens hole
        translate([CAM_ARM_LEN, 0, CAP_SOCK - 1]) cylinder(d = LENS_D, h = ARM_T + 2);
        // optional screw holes
        if (CAM_HOLES)
            for (sx = [-1, 1], sy = [-1, 1])
                translate([CAM_ARM_LEN + sx*CAM_HOLE_SP/2, sy*CAM_HOLE_SP/2, CAP_SOCK - 1])
                    cylinder(d = CAM_HOLE_D, h = ARM_T + 2, $fn = 24);
        // cable notch toward the free end of the arm
        if (CAM_CABLE)
            translate([CAM_ARM_LEN + CAM_POCKET/2 - 1, -6, top_z - 3.5]) cube([PAD, 12, 4]);
    }
}

// ---------- print orientations ----------
module base_print() { translate([0, 0, plate_x/2]) rotate([0, 90, 0]) base(); }
module rod_print()  { translate([0, 0, OUT/2]) rotate([0, 90, 0]) rod_segment(); }
module cap_print() {
    if (top_mode == "bolt") translate([0, 0, CAP_H]) rotate([180, 0, 0]) top_cap();
    else top_cap();
}
// L lying on its side: layers run along the arm, so it is strongest where it bends
module cam_arm_print() { translate([0, 0, PAD/2]) rotate([90, 0, 0]) cam_arm(); }

module top_assembled() { if (top_part == "cam_arm") cam_arm(); else top_cap(); }
module top_print()     { if (top_part == "cam_arm") cam_arm_print(); else cap_print(); }

// ---------- output ----------
if (part == "base") base_print();
else if (part == "rod") rod_print();
else if (part == "cap") cap_print();
else if (part == "cam_arm") cam_arm_print();
else if (part == "assembly") {
    color("DimGray") base();
    for (i = [0 : N - 1])
        color(i % 2 == 0 ? "SteelBlue" : "LightSteelBlue")
            translate([0, 0, BASE_H + i*SEG_LEN]) rod_segment();
    color("Orange") translate([0, 0, BASE_H + N*SEG_LEN]) top_assembled();
}
else {
    translate([-30, 0, 0]) base_print();
    for (i = [0 : N - 1]) translate([0, 60 + i*(OUT + 8), 0]) rod_print();
    translate([60, -80, 0]) top_print();
}

// ============================================================
//  NOTES
//  * Snap tongues flex in the layer plane - keep the print orientations as given.
//  * cam_arm: measure your camera board. Adjust CAM_SIZE, CAM_POCKET_D,
//    LENS_D, and (if you want screws) CAM_HOLES / CAM_HOLE_SP. If the lens
//    barrel is long, reduce the floor (raise CAM_POCKET_D or lower ARM_T).
//  * Pocket is 34.6 mm wide and opens upward in the assembly; on the printer
//    that face points sideways, so use supports there (or tree supports).
// ============================================================
