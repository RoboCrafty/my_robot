import socket
import math
import time
import json
from collections import deque
import matplotlib.pyplot as plt

# --- Robot connection --------------------------------------------------------
# ROBOT_IP = "127.0.0.1"
ROBOT_IP = "rpi.local"
ROBOT_PORT = 5005
robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(cmd: str):
    """Sends commands to the C++ backend"""
    robot_sock.sendto(cmd.encode("utf-8"), (ROBOT_IP, ROBOT_PORT))

def recv_state(timeout=0.1):
    """Return the newest state packet from the controller, or None on timeout."""
    robot_sock.settimeout(timeout)
    try:
        data, _ = robot_sock.recvfrom(4096)
    except socket.timeout:
        return None
    # Drain any backlog so the caller always gets the freshest state.
    robot_sock.setblocking(False)
    try:
        while True:
            data, _ = robot_sock.recvfrom(4096)
    except BlockingIOError:
        pass
    try:
        return json.loads(data)
    except json.JSONDecodeError:
        return None

send_cmd("ready")
time.sleep(1)
joint_angles = []
num_joints = 6

freq = 50
period = 1.0/freq
next_time = time.time() + period

sin_scale = 20.0

# --- Live plot ---------------------------------------------------------------
PLOT_JOINT = 0          # which joint (0-5) to plot
WINDOW_S = 10.0         # seconds of history shown
t0 = time.time()
ts, sent, actual, target = (deque() for _ in range(4))

plt.ion()
fig, ax = plt.subplots()
(l_sent,) = ax.plot([], [], label="target sent")
(l_tgt,) = ax.plot([], [], label="target pose")
(l_act,) = ax.plot([], [], label="actual pos")
ax.set_xlabel("time (s)")
ax.set_ylabel(f"J{PLOT_JOINT + 1} (deg)")
ax.legend(loc="upper right")
ax.grid(True)
redraw_count = 0

while True:
    # --- TIMING CONTROL ---
    # Check what time it is now, and sleep for the REMAINING time
    current_time = time.time()
    sleep_time = next_time - current_time
    
    # Only sleep if we finished the work before the 0.01s deadline
    if sleep_time > 0:
        time.sleep(sleep_time)
        
    # Schedule the exact time the next loop should finish
    next_time += period
    # send_cmd(f"cartjogvel x {vx:.4f}")
    
    # if state
    #     print("pos", state["pos"], "tcp", state["tcp"], "busy", state["busy"])

    target_angle = math.sin(current_time) * sin_scale
    send_cmd(f"{target_angle:.4f} {target_angle:.4f} {target_angle:.4f} {target_angle:.4f} {target_angle:.4f} {target_angle:.4f}")
    state = recv_state()
    if state is None:
        continue
    # print(f"Target sent: {target_angle:.2f}, actual pose received: {state["pos"]}, target pose: {state["tgt"]}")

    t = time.time() - t0
    ts.append(t)
    sent.append(target_angle)
    actual.append(state["pos"][PLOT_JOINT])
    target.append(state["tgt"][PLOT_JOINT])
    while ts and ts[0] < t - WINDOW_S:
        for d in (ts, sent, actual, target):
            d.popleft()

    # Redrawing every sample is slow; every 3rd keeps the loop on schedule.
    redraw_count += 1
    if redraw_count % 3 == 0:
        l_sent.set_data(ts, sent)
        l_tgt.set_data(ts, target)
        l_act.set_data(ts, actual)
        ax.set_xlim(max(0, t - WINDOW_S), t + 0.1)
        ax.relim()
        ax.autoscale_view(scalex=False)
        plt.pause(0.001)