#!/usr/bin/env python3
import json
import socket
import time
import tkinter as tk

import numpy as np

# --- Robot connection --------------------------------------------------------
# ROBOT_IP = "127.0.0.1"
ROBOT_IP = "rpi.local"
ROBOT_PORT = 5005
robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(cmd: str) -> None:
    robot_sock.sendto(cmd.encode("utf-8"), (ROBOT_IP, ROBOT_PORT))

def recv_state(timeout=0.1):
    robot_sock.settimeout(timeout)
    try:
        data, _ = robot_sock.recvfrom(4096)
        return json.loads(data)
    except (socket.timeout, json.JSONDecodeError):
        return None

# --- Settings ----------------------------------------------------------------
USER_YAW_OFFSET_DEG = -90.0
M_PER_PX = 0.0005         # pad pixels -> metres of TCP travel (300 px = 15 cm)
KP = 4.0                  # 1/s: commanded speed per metre of position error
VEL_CAP = 0.3             # m/s
DEADBAND_M = 0.0005
TICK_HZ = 500.0

theta = np.radians(USER_YAW_OFFSET_DEG)
c, s = np.cos(theta), np.sin(theta)
YAW_ROTATION_MATRIX = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# Same mapping as teleop_leap.py: leap (x right, y up, z toward user) -> robot base.
def _leap_to_robot(v: np.ndarray) -> np.ndarray:
    return np.array([-v[2], -v[0], v[1]])

# Pad (right, down) pixels -> robot base XY offset in metres.
def _pad_to_robot(px: float, py: float) -> np.ndarray:
    v = YAW_ROTATION_MATRIX.dot(_leap_to_robot(np.array([px, 0.0, py])))
    return v[:2] * M_PER_PX

PAD_TO_ROBOT = np.column_stack([_pad_to_robot(1, 0), _pad_to_robot(0, 1)])
ROBOT_TO_PAD = np.linalg.inv(PAD_TO_ROBOT)

# --- Window ------------------------------------------------------------------
SIZE = 600
READY_TIMEOUT_S = 20.0

class TrackpadPad:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Trackpad teleop")
        self.canvas = tk.Canvas(self.root, width=SIZE, height=SIZE, bg="#111", highlightthickness=0)
        self.canvas.pack()
        c0 = SIZE // 2
        self.canvas.create_line(c0, 0, c0, SIZE, fill="#333")
        self.canvas.create_line(0, c0, SIZE, c0, fill="#333")
        self.dot = self.canvas.create_oval(c0 - 8, c0 - 8, c0 + 8, c0 + 8, outline="#888", width=2)
        self.tcp_dot = self.canvas.create_oval(c0 - 6, c0 - 6, c0 + 6, c0 + 6, fill="#3c3", outline="")
        self.text = self.canvas.create_text(10, 10, anchor="nw", fill="#ddd", font=("Menlo", 13), text="")

        self.held = False
        self.was_active = False
        self.cursor = None          # pad pixels relative to centre (right, down)
        self.origin = None          # TCP xy at the ready pose = pad centre
        self.tcp = None
        self.vx = self.vy = 0.0
        self.ready = False
        self.ready_start = time.time()

        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Leave>", lambda e: setattr(self, "cursor", None))
        self.root.bind("<KeyPress-space>", lambda e: setattr(self, "held", True))
        self.root.bind("<KeyRelease-space>", lambda e: setattr(self, "held", False))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.focus_force()

    def on_motion(self, e):
        self.cursor = np.array([e.x - SIZE // 2, e.y - SIZE // 2], dtype=float)
        self.canvas.coords(self.dot, e.x - 8, e.y - 8, e.x + 8, e.y + 8)

    def stop_robot(self):
        send_cmd("cartjogvel x 0.0")
        send_cmd("cartjogvel y 0.0")
        self.vx = self.vy = 0.0

    def close(self):
        self.stop_robot()
        self.root.destroy()

    def tick(self):
        now = time.time()
        state = recv_state(0.001)
        if state:
            self.tcp = np.array(state["tcp"])

        if not self.ready:
            waited = now - self.ready_start
            if state and ((not state["busy"] and waited > 0.5) or waited > READY_TIMEOUT_S):
                self.origin = self.tcp[:2].copy()
                self.ready = True
            status = "moving to ready pose..."
        else:
            active = self.held and self.cursor is not None and self.tcp is not None
            status = "ACTIVE" if active else "hold SPACE and move the cursor here"
            if active:
                target = self.origin + PAD_TO_ROBOT.dot(self.cursor)
                err = target - self.tcp[:2]
                dist = np.linalg.norm(err)
                if dist < DEADBAND_M:
                    vel = np.zeros(2)
                else:
                    vel = err * KP
                    speed = np.linalg.norm(vel)
                    if speed > VEL_CAP:
                        vel *= VEL_CAP / speed
                self.vx, self.vy = vel
                send_cmd(f"cartjogvel x {self.vx:.4f}")
                send_cmd(f"cartjogvel y {self.vy:.4f}")
            elif self.was_active:
                self.stop_robot()
            self.was_active = active

            if self.tcp is not None:
                px, py = ROBOT_TO_PAD.dot(self.tcp[:2] - self.origin) + SIZE // 2
                self.canvas.coords(self.tcp_dot, px - 6, py - 6, px + 6, py + 6)

        self.canvas.itemconfig(self.text, text=f"{status}\nvx {self.vx:+.3f}  vy {self.vy:+.3f} m/s")
        self.root.after(int(1000 / TICK_HZ), self.tick)

    def run(self):
        send_cmd("cartframe base")
        send_cmd("ready")
        self.tick()
        self.root.mainloop()

if __name__ == "__main__":
    TrackpadPad().run()
