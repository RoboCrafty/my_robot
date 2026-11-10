#!/usr/bin/env python3
"""Teleoperate the Pinocchio arm controller with the custom "Robot Arm Teleoperator"
iOS app (from /Users/fudayl/git/robot-arm, flashed via Xcode) instead of the HEBI
Mobile I/O app used by teleop.py -- HEBI's built-in ARKit tracking proved unreliable
(ar_quality stuck at NotAvailable) on this phone/app combo.

In the iOS app, set the server host to this machine's IP and APP_PORT below, then
press Move to start streaming pose deltas; release Move to stop. Swipe the gripper
strip to open/close the gripper.
"""

import json
import socket
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

# --- Robot connection --------------------------------------------------------
ROBOT_IP = "192.168.87.168"
ROBOT_PORT = 5005
robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_cmd(cmd: str) -> None:
    robot_sock.sendto(cmd.encode("utf-8"), (ROBOT_IP, ROBOT_PORT))


# --- iOS app connection -------------------------------------------------------
APP_PORT = 8000
GRIPPER_POS_MAX = 140  # see pinocchio/src/protocol.h
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])  # scipy quat order: x, y, z, w


class PhoneTeleop:
    """Receives framed JSON pose packets from the iOS app over TCP+UDP.

    Same protocol as /Users/fudayl/git/robot-arm/teleop_phone.py: every message is
    a 4-byte little-endian total-length prefix followed by JSON. A TCP connection
    must be open before the app's Move button activates; pose updates then stream
    over UDP to the same port.
    """

    def __init__(self, port: int = APP_PORT):
        self._port = port
        self._lock = threading.Lock()
        self._connected = False
        self._enabled = False
        self._position = np.zeros(3)
        self._world_rotation = Rotation.from_quat(IDENTITY_QUAT)
        self._gripper = 0.0

    def connect(self) -> None:
        threading.Thread(target=self._run_tcp, daemon=True).start()
        threading.Thread(target=self._run_udp, daemon=True).start()

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def get_state(self):
        """Returns (enabled, position, world_rotation, gripper_open_0_1)."""
        with self._lock:
            return self._enabled, self._position, self._world_rotation, self._gripper

    def _handle_message(self, msg: dict) -> None:
        message_id = msg.get("__id")

        if message_id == "PoseStateMessage":
            position = np.asarray(msg.get("gripperDeltaPosition", [0.0, 0.0, 0.0]), dtype=float)
            quat = np.asarray(msg.get("deviceWorldRotation", IDENTITY_QUAT), dtype=float)
            norm = float(np.linalg.norm(quat))
            with self._lock:
                self._position = position
                if norm > 1e-6:
                    self._world_rotation = Rotation.from_quat(quat / norm)
                self._gripper = float(msg.get("gripperOpenAmount", 0.0))

        elif message_id == "BeginEpisodeMessage":
            with self._lock:
                self._enabled = True
            print("Move pressed: teleop enabled.")

        elif message_id == "EndEpisodeMessage":
            with self._lock:
                self._enabled = False
            print("Move released: teleop disabled.")

    def _run_tcp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self._port))
        sock.listen(1)
        print(f"Waiting for the iOS app on TCP port {self._port}...")
        while True:
            conn, addr = sock.accept()
            print(f"Phone connected: {addr[0]}")
            with self._lock:
                self._connected = True
            try:
                with conn:
                    while True:
                        header = _read_exactly(conn, 4)
                        if header is None:
                            break
                        total_size = int.from_bytes(header, byteorder="little")
                        if total_size < 4:
                            break
                        body = _read_exactly(conn, total_size - 4)
                        if body is None:
                            break
                        msg = _decode_frame(header + body)
                        if msg is not None:
                            self._handle_message(msg)
            except OSError:
                pass
            with self._lock:
                self._connected = False
                self._enabled = False
            print("Phone disconnected")

    def _run_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self._port))
        while True:
            data, _ = sock.recvfrom(4096)
            msg = _decode_frame(data)
            if msg is not None:
                self._handle_message(msg)


def _read_exactly(conn: socket.socket, count: int):
    buffer = b""
    while len(buffer) < count:
        chunk = conn.recv(count - len(buffer))
        if not chunk:
            return None
        buffer += chunk
    return buffer


def _decode_frame(data: bytes):
    if len(data) < 4:
        return None
    total_size = int.from_bytes(data[0:4], byteorder="little")
    if total_size < 4 or total_size > len(data):
        return None
    try:
        return json.loads(data[4:total_size].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


# --- Teleop main loop ----------------------------------------------------------
POSITION_GAIN = 1.0   # m of robot travel per m of phone travel
ROTATION_GAIN = 1.0   # rad of robot rotation per rad of phone rotation
RATE_HZ = 50.0
VEL_CAP = 0.5         # m/s python-side clip (firmware's own cartvel caps further)
ANG_VEL_CAP = 1.0     # rad/s python-side clip (firmware's own cartrotvel caps further)
SMOOTHING = 0.8      # 0 = raw/jittery, close to 1 = heavily damped/laggy
GRIPPER_EPSILON = 1   # only re-send `gripper` when it moves by more than this (0..140)

if __name__ == "__main__":
    # Velocities below are aligned to a fixed reference frame captured at Move-press,
    # so this only makes sense with the world/base cartesian frame, not "tool".
    send_cmd("cartframe base")

    phone = PhoneTeleop(port=APP_PORT)
    phone.connect()
    print(f"In the iOS app, set Server host to this machine's IP and port {APP_PORT}.")

    # Numerically differentiating position/rotation cancels out any constant offset
    # the app's pose reference happens to use, so there's no need to track an origin
    # ourselves for translation. Rotation direction is aligned to home_rotation --
    # captured fresh every time Move is pressed -- same technique as teleop.py.
    last_pos = None
    last_rot = None
    last_time = None
    home_rotation = Rotation.identity()
    was_enabled = False
    print_counter = 0
    smoothed_vel = np.zeros(3)
    smoothed_ang_vel = np.zeros(3)
    last_gripper_cmd = None
    while True:
            loop_start = time.time()
            enabled, pos, rot, gripper_open = phone.get_state()

            if phone.is_connected:
                gripper_cmd = int(np.clip(gripper_open, 0.0, 1.0) * GRIPPER_POS_MAX)
                if last_gripper_cmd is None or abs(gripper_cmd - last_gripper_cmd) > GRIPPER_EPSILON:
                    send_cmd(f"gripper {gripper_cmd}")
                    last_gripper_cmd = gripper_cmd

            if enabled:
                if not was_enabled:
                    home_rotation = rot.inv()
                    last_pos, last_rot, last_time = pos, rot, loop_start
                    smoothed_vel[:] = 0.0
                    smoothed_ang_vel[:] = 0.0
                else:
                    dt = loop_start - last_time
                    if dt >= 0.005:
                        raw_vel = (pos - last_pos) / dt
                        delta_rotation = rot * last_rot.inv()
                        raw_ang_vel = delta_rotation.as_rotvec() / dt

                        # 1. Rotate the raw vectors into the "Homed" reference frame
                        aligned_vel = home_rotation.apply(raw_vel)
                        aligned_ang_vel = home_rotation.apply(raw_ang_vel)

                        # 2. Map Phone Coordinates to Robot Base Coordinates
                        # Phone Flat: +Y is Forward, +X is Right, +Z is Up
                        # Robot Base: +X is Forward, +Y is Left,  +Z is Up
                        robot_vel = np.array([
                            aligned_vel[1],      # Robot X = Phone Y
                            -aligned_vel[0],     # Robot Y = -Phone X
                            aligned_vel[2]       # Robot Z = Phone Z
                        ]) * POSITION_GAIN

                        robot_ang_vel = np.array([
                            aligned_ang_vel[1],  # Robot RX = Phone RY
                            -aligned_ang_vel[0], # Robot RY = -Phone RX
                            aligned_ang_vel[2]   # Robot RZ = Phone RZ
                        ]) * ROTATION_GAIN

                        alpha = 1.0 - SMOOTHING
                        smoothed_vel += alpha * (robot_vel - smoothed_vel)
                        smoothed_ang_vel += alpha * (robot_ang_vel - smoothed_ang_vel)

                        vx, vy, vz = np.clip(smoothed_vel, -VEL_CAP, VEL_CAP)
                        wx, wy, wz = np.clip(smoothed_ang_vel, -ANG_VEL_CAP, ANG_VEL_CAP)

                        # 3. Network Deadband: Only blast UDP packets if actually moving
                        # This prevents the script from starving the Web UI's telemetry
                        is_moving_translation = np.linalg.norm([vx, vy, vz]) > 0.005
                        is_moving_rotation = np.linalg.norm([wx, wy, wz]) > 0.005

                        if is_moving_translation or is_moving_rotation:
                            send_cmd(f"cartjogvel x {vx:.4f}")
                            send_cmd(f"cartjogvel y {vy:.4f}")
                            send_cmd(f"cartjogvel z {vz:.4f}")
                            send_cmd(f"cartjogvel rx {wx:.4f}")
                            send_cmd(f"cartjogvel ry {wy:.4f}")
                            send_cmd(f"cartjogvel rz {wz:.4f}")

                        print_counter += 1
                        if print_counter % 10 == 0:
                            print(f"dt={dt:.4f} v=({vx:+.3f},{vy:+.3f},{vz:+.3f}) "
                                f"w=({wx:+.3f},{wy:+.3f},{wz:+.3f})")

                        last_pos, last_rot, last_time = pos, rot, loop_start
            elif was_enabled:
                for axis in ("x", "y", "z", "rx", "ry", "rz"):
                    send_cmd(f"cartjogvel {axis} 0.0")

            was_enabled = enabled
            time.sleep(max(0.0, 1.0 / RATE_HZ - (time.time() - loop_start)))