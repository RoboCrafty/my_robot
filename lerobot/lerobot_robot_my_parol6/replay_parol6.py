#!/usr/bin/env python3
"""Replay a recorded episode's actions open-loop -- the "does the recorded data actually reproduce the demo" check.

Moves the arm to the episode's start pose, (with --sim) puts the cubes back where they were at the start of that
episode, then streams the recorded actions at the dataset fps through the same `send_action` a policy would use.
Afterwards it prints how closely the replayed joints tracked the recorded ones and, in sim, whether the green cube
ended up on the red one.

Cube start poses come from meta/sim_cube_poses.json (written by record_parol6.py) when available, otherwise they are
estimated from the episode's first top_down_cam frame.
"""

import argparse
import json
import math
import socket
import time
from pathlib import Path

import numpy as np

SCENE_CMD_ADDR = ("127.0.0.1", 5006)  # mj_sim.py
CUBE_HALF = 0.015
# top_down_cam in parol6-scene1.xml: pos (0, 0, 1), zaxis (0, 0, 1) -> camera axes == world axes, looking straight down
CAM_HEIGHT = 1.0
CAM_FOVY_DEG = 75.0


def detect_cube(rgb: np.ndarray, name: str):
    """[x, y, z, yaw] of the red/green cube resting on the table in a top_down_cam RGB frame, or None if not fully visible."""
    import cv2

    h, w = rgb.shape[:2]
    f = (h / 2) / math.tan(math.radians(CAM_FOVY_DEG) / 2)
    depth = CAM_HEIGHT - 2 * CUBE_HALF  # camera to the cube's top face
    r, g, b = (rgb[..., i].astype(int) for i in range(3))
    mask = ((r > 120) & (r > 2 * g) & (r > 2 * b)) if name == "red" else ((g > 100) & (g > 2 * r) & (g > 1.5 * b))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    full_area = (2 * CUBE_HALF / depth * f) ** 2
    if stats[k, cv2.CC_STAT_AREA] < 0.7 * full_area:  # partly hidden under the arm
        return None
    vs, us = np.nonzero(labels == k)
    (cu, cv_), _, angle = cv2.minAreaRect(np.column_stack([us, vs]).astype(np.float32))
    # image u -> world +x, image v (down) -> world -y
    x = (cu + 0.5 - w / 2) / f * depth
    y = -(cv_ + 0.5 - h / 2) / f * depth
    return [x, y, CUBE_HALF, -math.radians(angle)]


def scene_cmd(sock, cmd: str):
    sock.sendto(cmd.encode(), SCENE_CMD_ADDR)
    v = [float(t) for t in sock.recv(256).decode().split()[1:]]
    return {"red": v[:4], "green": v[4:]}


def wait_idle(robot):
    time.sleep(0.3)  # let the controller flag the move as busy
    while robot.get_status()["busy"]:
        time.sleep(0.05)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo-id", required=True)
    p.add_argument("--episode", type=int, required=True)
    p.add_argument("--root", default=None)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--sim", action="store_true", help="restore the episode's cube poses in mj_sim and check the stack")
    args = p.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot_robot_my_parol6 import myParol6, myParol6Config

    ds = LeRobotDataset(args.repo_id, root=args.root)
    rows = np.flatnonzero(np.asarray(ds.hf_dataset["episode_index"]) == args.episode)
    if len(rows) == 0:
        raise SystemExit(f"episode {args.episode} not in dataset ({ds.meta.total_episodes} episodes)")
    actions = np.stack([np.asarray(ds.hf_dataset[int(i)]["action"]) for i in rows])
    states = np.stack([np.asarray(ds.hf_dataset[int(i)]["observation.state"]) for i in rows])
    names = ds.features["action"]["names"]
    print(f"episode {args.episode}: {len(rows)} frames ({len(rows) / ds.fps:.1f}s at {ds.fps} fps)")

    scene = None
    if args.sim:
        scene = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        scene.settimeout(1.0)
        poses_file = Path(ds.root) / "meta" / "sim_cube_poses.json"
        recorded = json.loads(poses_file.read_text()) if poses_file.exists() else {}
        if str(args.episode) in recorded:
            cube_poses = recorded[str(args.episode)]
            print("cube start poses: recorded")
        else:
            # Cubes can't move before the grasp (the gripper's tightest closure), so any earlier frame shows the start pose.
            grip = actions[:, names.index("gripper.pos")]
            last = int(np.argmax(grip > 0.8 * grip.max())) if grip.max() > 0.1 else len(rows)
            cube_poses = {}
            for i in range(0, last, 3):
                img = ds[int(rows[i])]["observation.images.top_down_cam"]
                rgb = (np.asarray(img).transpose(1, 2, 0) * 255).astype(np.uint8)
                for name in ("red", "green"):
                    if name in cube_poses:
                        continue
                    # episodes recorded before the mj_sim color fix have red/blue swapped
                    pose = detect_cube(rgb, name) or detect_cube(rgb[..., ::-1], name)
                    if pose is not None:
                        cube_poses[name] = pose
                if len(cube_poses) == 2:
                    break
            missing = {"red", "green"} - set(cube_poses)
            if missing:
                raise SystemExit(f"{missing} cube hidden under the arm in every pre-grasp frame; can't restore the scene")
            print("cube start poses: estimated from top_down_cam")
        for k, v in cube_poses.items():
            print(f"  {k:5s} x={v[0]:.3f} y={v[1]:.3f} yaw={math.degrees(v[3]):.0f} deg")

    robot = myParol6(myParol6Config(host=args.host, cameras={}))
    robot.connect()
    try:
        start = dict(zip(names, states[0]))
        print("moving to the episode's start pose...")
        robot.send_action(start)
        wait_idle(robot)
        if scene:
            scene_cmd(scene, "place " + " ".join(f"{c:.5f}" for k in ("red", "green") for c in cube_poses[k]))
            time.sleep(0.5)  # let the cubes settle

        print("replaying...")
        replayed = []
        period = 1.0 / ds.fps
        next_tick = time.perf_counter()
        for a in actions:
            robot.send_action(dict(zip(names, a)))
            obs = robot.get_observation()
            replayed.append([obs[k] for k in names])
            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))
        wait_idle(robot)
        replayed = np.asarray(replayed)

        err = np.abs(replayed - states)
        np.set_printoptions(precision=2, suppress=True)
        print("\ntracking vs recorded state (joints in deg, gripper 0..1):")
        print("  joints:   ", names)
        print("  mean err: ", err.mean(0))
        print("  max err:  ", err.max(0))
        print("  final pose replayed:", replayed[-1], "\n  final pose recorded:", states[-1])

        if scene:
            end = scene_cmd(scene, "poses")
            red, green = np.array(end["red"]), np.array(end["green"])
            dxy = np.linalg.norm(green[:2] - red[:2])
            stacked = dxy < 0.015 and abs(green[2] - red[2] - 2 * CUBE_HALF) < 0.006
            print(f"\ncubes at end: green {green[:3].round(3)}, red {red[:3].round(3)}, xy offset {dxy * 1000:.0f} mm")
            print("STACKED" if stacked else "not stacked")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
