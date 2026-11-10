import argparse
import base64
import mujoco
import mujoco.viewer
import socket
import json
import random
import threading
import time
import math

import cv2
import numpy as np
import zmq

# Shared state between the network thread and MuJoCo thread
running = True
latest_targets = [0.0] * 6
latest_gripper = 0.0
ROBOT_IP = "127.0.0.1"
# ROBOT_IP = "rpi.local"
def udp_telemetry_listener():
    """Listens to the C++ app's 31Hz JSON telemetry broadcast."""
    global latest_targets, latest_gripper
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))  # local address; the Pi replies to whatever port we send from
    sock.settimeout(0.1)
    
    # Resolve once: "rpi.local" goes through mDNS, which is slow to repeat per packet
    server_addr = (socket.gethostbyname(ROBOT_IP), 5005)
    sock.sendto(b"ping", server_addr)
    last_ping_time = 0
    
    while running:
        if time.time() - last_ping_time > 2.0:
            sock.sendto(b"ping", server_addr)
            last_ping_time = time.time()
        try:
            data, _ = sock.recvfrom(2048)
            msg = json.loads(data.decode('utf-8'))
            # print(msg)
            
            # The C++ app sends angles in degrees, MuJoCo uses radians.
            # Using msg["tgt"] (the Ruckig output) to drive the actuators
            latest_targets = [math.radians(deg) for deg in msg["pos"]]
            
            # Gripper is 0-140 in C++. MuJoCo left_jaw_joint is 0.0 to 0.027.
            latest_gripper = 0.027 - ((msg["grip"] / 140.0) * 0.027)
            
        except socket.timeout:
            # Re-register if the C++ app restarts or drops us
            sock.sendto(b"ping", server_addr)
        except json.JSONDecodeError:
            pass
            
    sock.close()

CAMERAS = ("top_down_cam", "wrist_cam")
CAM_W, CAM_H, CAM_FPS = 640, 480, 30
CAM_PORT = 5555      # lerobot ZMQCamera protocol
SCENE_CMD_PORT = 5006
CUBE_JOINTS = ("cube_joint", "green_cube_joint")  # red, green
CUBE_HALF = 0.015


class FramePublisher:
    """JPEG-encodes and publishes frames off the physics thread, in lerobot's ZMQCamera format."""

    def __init__(self, port):
        self.sock = zmq.Context.instance().socket(zmq.PUB)
        self.sock.bind(f"tcp://127.0.0.1:{port}")
        self._latest = None
        self._cv = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def submit(self, frames, stamp):
        with self._cv:
            self._latest = (frames, stamp)
            self._cv.notify()

    def _run(self):
        while running:
            with self._cv:
                self._cv.wait_for(lambda: self._latest is not None or not running, timeout=0.5)
                item, self._latest = self._latest, None
            if item is None:
                continue
            frames, stamp = item
            msg = {"timestamps": {}, "images": {}}
            for name, rgb in frames.items():
                # lerobot's ZMQCamera returns cv2.imdecode output unconverted, so encode the RGB array as-is
                ok, buf = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ok:
                    msg["timestamps"][name] = stamp
                    msg["images"][name] = base64.b64encode(buf).decode("ascii")
            self.sock.send_string(json.dumps(msg), zmq.NOBLOCK)


def set_cube_poses(model, data, poses):
    """poses: [(x, y, z, yaw), ...] for (red, green)."""
    for joint, (x, y, z, yaw) in zip(CUBE_JOINTS, poses):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        qa, va = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
        data.qpos[qa:qa + 7] = [x, y, z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        data.qvel[va:va + 6] = 0.0
    mujoco.mj_forward(model, data)


def reset_cubes(model, data, stacked):
    """Random non-overlapping cube poses in front of the robot; `stacked` puts a random one on top."""
    def rand_xy():
        return np.array([random.uniform(0.18, 0.32), random.uniform(-0.12, 0.12)])

    if stacked:
        xy = rand_xy()
        bottom, top = random.sample(range(2), 2)
        placed = {bottom: (xy, CUBE_HALF), top: (xy, 3 * CUBE_HALF + 0.001)}
    else:
        a = rand_xy()
        b = rand_xy()
        while np.linalg.norm(a - b) < 0.07:
            b = rand_xy()
        placed = {0: (a, CUBE_HALF), 1: (b, CUBE_HALF)}

    poses = [(float(placed[i][0][0]), float(placed[i][0][1]), placed[i][1], random.uniform(-math.pi, math.pi))
             for i in range(2)]
    set_cube_poses(model, data, poses)
    return poses


def main():
    global running

    model = mujoco.MjModel.from_xml_path("assets/parol6-scene1.xml")
    data = mujoco.MjData(model)

    listener_thread = threading.Thread(target=udp_telemetry_listener)
    listener_thread.daemon = True
    listener_thread.start()

    renderer = mujoco.Renderer(model, CAM_H, CAM_W)
    publisher = FramePublisher(CAM_PORT)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(("127.0.0.1", SCENE_CMD_PORT))
    cmd_sock.setblocking(False)

    print(f"Publishing {CAMERAS} on tcp://127.0.0.1:{CAM_PORT}; scene commands on udp {SCENE_CMD_PORT}.")
    try:
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            t_start = time.perf_counter()
            sim_start = data.time
            next_view = next_cam = 0.0

            while viewer.is_running():
                try:
                    raw, sender = cmd_sock.recvfrom(256)
                    cmd = raw.decode().split()
                    poses = None
                    if cmd and cmd[0] == "reset":
                        poses = reset_cubes(model, data, stacked="stacked" in cmd)
                    elif cmd and cmd[0] == "place" and len(cmd) == 9:
                        # place rx ry rz ryaw gx gy gz gyaw  (red, green; m and rad)
                        v = [float(t) for t in cmd[1:]]
                        poses = [tuple(v[:4]), tuple(v[4:])]
                        set_cube_poses(model, data, poses)
                    elif cmd and cmd[0] == "poses":
                        poses = []
                        for joint in CUBE_JOINTS:
                            qa = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)]
                            x, y, z, qw, _, _, qz = data.qpos[qa:qa + 7]
                            poses.append((x, y, z, 2 * math.atan2(qz, qw)))
                    if poses is not None:
                        reply = "poses " + " ".join(f"{c:.5f}" for p in poses for c in p)
                        cmd_sock.sendto(reply.encode(), sender)
                except BlockingIOError:
                    pass

                # Actuators 0-5 are the arm, actuator 6 is left_jaw_joint
                data.ctrl[0:6] = latest_targets
                data.ctrl[6] = latest_gripper

                # Step until sim time catches up with wall time, so rendering/viewer cost doesn't slow physics
                wall = time.perf_counter() - t_start
                while data.time - sim_start < wall:
                    mujoco.mj_step(model, data)

                now = time.perf_counter()
                if now >= next_cam:
                    next_cam = now + 1.0 / CAM_FPS
                    frames = {}
                    for cam in CAMERAS:
                        renderer.update_scene(data, camera=cam)
                        frames[cam] = renderer.render().copy()
                    publisher.submit(frames, time.time())
                if now >= next_view:
                    next_view = now + 1.0 / 60
                    viewer.sync()

                time.sleep(0.0005)
    except KeyboardInterrupt:
        # FIX 2: Catch Ctrl+C gracefully
        print("\nReceived exit signal. Shutting down MuJoCo...")
        
    finally:
        # Guarantee cleanup runs no matter how the loop exits
        running = False
        listener_thread.join(timeout=1.0)
        print("Shutdown complete.")
                
    # Shutdown
    running = False
    listener_thread.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=ROBOT_IP, help="controller address (127.0.0.1 for the --sim controller)")
    ROBOT_IP = parser.parse_args().host
    main()



    