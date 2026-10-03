#!/usr/bin/env python3
import socket
import threading
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation
import leap
from pynput import keyboard

# --- Robot connection --------------------------------------------------------
# ROBOT_IP = "192.168.50.10"
ROBOT_IP = "rpi.local"
ROBOT_PORT = 5005
robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(cmd: str) -> None:
    robot_sock.sendto(cmd.encode("utf-8"), (ROBOT_IP, ROBOT_PORT))

# --- Perspective & Settings --------------------------------------------------
# Change this based on where you are sitting relative to the robot's base!
# 0 = Behind, 180 = In front (facing robot), 90 = Left side, -90 = Right side
USER_YAW_OFFSET_DEG = -90.0 

# Gripper distance tuning (in millimeters)
# Measure roughly how far apart your fingers are when you want it "Open" vs "Closed"
FINGER_DIST_OPEN = 100.0   # mm
FINGER_DIST_CLOSED = 25.0 # mm

# --- Keyboard State ----------------------------------------------------------
key_state = {"move_pressed": False}

def on_key_press(key):
    if key == keyboard.Key.space:
        key_state["move_pressed"] = True

def on_key_release(key):
    if key == keyboard.Key.space:
        key_state["move_pressed"] = False

keyboard_listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
keyboard_listener.start()

# --- Leap Motion connection --------------------------------------------------
GRIPPER_POS_MAX = 140
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])

class LeapTeleop(leap.Listener):
    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._enabled = False
        self._position = np.zeros(3)
        self._world_rotation = Rotation.from_quat(IDENTITY_QUAT)
        self._target_gripper = 0.0

    def on_tracking_event(self, event):
        with self._lock:
            if len(event.hands) == 1 and key_state["move_pressed"]:
                hand = event.hands[0]
                self._enabled = True
                
                self._position = np.array([
                    hand.palm.position.x,
                    hand.palm.position.y,
                    hand.palm.position.z
                ]) / 1000.0
                
                self._world_rotation = Rotation.from_quat([
                    hand.palm.orientation.x,
                    hand.palm.orientation.y,
                    hand.palm.orientation.z,
                    hand.palm.orientation.w
                ])
                
                # --- NEW GRIPPER MATH (Physical Distance) ---
                # Digits: 0=Thumb, 1=Index. Get the distal joint (tip) position
                thumb_tip = hand.digits[0].distal.next_joint
                index_tip = hand.digits[1].distal.next_joint
                
                # Calculate physical distance in millimeters
                dist = math.sqrt(
                    (thumb_tip.x - index_tip.x)**2 +
                    (thumb_tip.y - index_tip.y)**2 +
                    (thumb_tip.z - index_tip.z)**2
                )
                
                # Map the distance to a 0.0 to 1.0 scale, and clamp it so it never exceeds boundaries
                normalized_grip = (dist - FINGER_DIST_CLOSED) / (FINGER_DIST_OPEN - FINGER_DIST_CLOSED)
                normalized_grip = max(0.0, min(1.0, normalized_grip))
                
                # Invert if necessary (1.0 = open, 0.0 = closed) based on your robot's preference
                self._target_gripper = normalized_grip 
            else:
                self._enabled = False

    def get_state(self):
        with self._lock:
            return self._enabled, self._position.copy(), self._world_rotation, self._target_gripper

# --- Coordinate Frame Mapping ------------------------------------------------
def _leap_to_robot(v: np.ndarray) -> np.ndarray:
    return np.array([-v[2], -v[0], v[1]])

# Create the perspective rotation matrix
theta = np.radians(USER_YAW_OFFSET_DEG)
c, s = np.cos(theta), np.sin(theta)
YAW_ROTATION_MATRIX = np.array([
    [c, -s, 0],
    [s,  c, 0],
    [0,  0, 1]
])

# --- Teleop main loop --------------------------------------------------------
POSITION_GAIN = 1.0
ROTATION_GAIN = 0.6  
RATE_HZ = 50.0
VEL_CAP = 0.5
ANG_VEL_CAP = 2.0
SMOOTHING = 0.5
GRIPPER_SMOOTHING = 0.85  # <--- NEW: High smoothing specifically to protect the servo
GRIPPER_EPSILON = 1
MOTION_DEADBAND = 0.005

def main():
    send_cmd("cartframe base")

    leap_tracker = LeapTeleop()
    connection = leap.Connection()
    connection.add_listener(leap_tracker)

    last_pos = None
    last_rot = None
    last_time = None
    was_enabled = False
    
    smoothed_vel = np.zeros(3)
    smoothed_ang_vel = np.zeros(3)
    
    smoothed_gripper_val = 0.0 # Starts closed
    last_gripper_cmd = None

    print(f"\n[SYSTEM] Ready. Perspective Yaw Offset is set to {USER_YAW_OFFSET_DEG}°")
    print("[SYSTEM] HOLD [Spacebar] to steer the arm. Pinch index and thumb to grip.\n")

    with connection.open():
        try:
            while True:
                loop_start = time.time()
                enabled, pos, rot, target_gripper = leap_tracker.get_state()

                # --- NEW GRIPPER SMOOTHING ---
                # Apply an Exponential Moving Average to protect the servo from jitter
                smoothed_gripper_val = (GRIPPER_SMOOTHING * smoothed_gripper_val) + ((1.0 - GRIPPER_SMOOTHING) * target_gripper)
                
                gripper_cmd = 140 - int(np.clip(smoothed_gripper_val, 0.0, 1.0) * GRIPPER_POS_MAX)
                if last_gripper_cmd is None or abs(gripper_cmd - last_gripper_cmd) > GRIPPER_EPSILON:
                    send_cmd(f"gripper {gripper_cmd}")
                    last_gripper_cmd = gripper_cmd

                # --- Arm Control ---
                if enabled:
                    if not was_enabled:
                        last_pos, last_rot, last_time = pos, rot, loop_start
                        smoothed_vel = np.zeros(3)
                        smoothed_ang_vel = np.zeros(3)
                        print("[+] Tracking Active")
                    else:
                        dt = loop_start - last_time
                        if dt >= 0.005:
                            raw_vel = (pos - last_pos) / dt
                            delta_rotation = rot * last_rot.inv()
                            raw_ang_vel = delta_rotation.as_rotvec() / dt

                            # Leap is fixed on the desk, so its frame is already the stable reference;
                            # re-basing on the hand's pose at clutch would rotate the axes on every re-clutch.
                            base_robot_vel = _leap_to_robot(raw_vel) * POSITION_GAIN
                            base_robot_ang_vel = _leap_to_robot(raw_ang_vel) * ROTATION_GAIN
                            
                            # 2. Rotate the frame based on where you are sitting in the room
                            robot_vel = YAW_ROTATION_MATRIX.dot(base_robot_vel)
                            robot_ang_vel = YAW_ROTATION_MATRIX.dot(base_robot_ang_vel)

                            # 3. Apply smoothing to velocities
                            alpha = 1.0 - SMOOTHING
                            smoothed_vel += alpha * (robot_vel - smoothed_vel)
                            smoothed_ang_vel += alpha * (robot_ang_vel - smoothed_ang_vel)

                            vx, vy, vz = np.clip(smoothed_vel, -VEL_CAP, VEL_CAP)
                            wx, wy, wz = np.clip(smoothed_ang_vel, -ANG_VEL_CAP, ANG_VEL_CAP) 

                            if (np.linalg.norm([vx, vy, vz]) > MOTION_DEADBAND
                                    or np.linalg.norm([wx, wy, wz]) > MOTION_DEADBAND):
                                send_cmd(f"cartjogvel x {vx:.4f}")
                                send_cmd(f"cartjogvel y {vy:.4f}")
                                send_cmd(f"cartjogvel z {vz:.4f}")
                                send_cmd(f"cartjogvel rx {wx*1:.4f}")
                                send_cmd(f"cartjogvel ry {wy*1:.4f}")
                                send_cmd(f"cartjogvel rz {wz:.4f}")

                            last_pos, last_rot, last_time = pos, rot, loop_start
                
                elif was_enabled:
                    for axis in ("x", "y", "z", "rx", "ry", "rz"):
                        send_cmd(f"cartjogvel {axis} 0.0")
                    print("[-] Robot Halted")

                was_enabled = enabled
                time.sleep(max(0.0, 1.0 / RATE_HZ - (time.time() - loop_start)))

        except KeyboardInterrupt:
            for axis in ("x", "y", "z", "rx", "ry", "rz"):
                send_cmd(f"cartjogvel {axis} 0.0")

if __name__ == "__main__":
    main()