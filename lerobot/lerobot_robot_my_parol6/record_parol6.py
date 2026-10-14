#!/usr/bin/env python3
"""Record LeRobot demos of PAROL6 while you teleop it from another process (teleop_leap.py / phone app).

The teleop script drives the controller; this script only watches. Each tick it saves the camera images +
measured joints (`pos`, `grip_ack`) as the observation, and the controller's commanded IK joint target +
gripper command (`tgt`, `grip`) as the action -- no time-shifting needed.

Keys (global, so they work while the teleop window has focus):
    Right arrow  start recording / stop and save the episode
    Left arrow   discard the episode being recorded
    Esc          finish and write the dataset

After every episode the arm goes to `ready` and the gripper opens; with --sim the cubes are re-randomised.
Release the teleop clutch before stopping an episode, or the teleop will fight the `ready` move.
"""

import argparse
import json
import socket
import threading
import time
from pathlib import Path

from pynput import keyboard

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot_robot_my_parol6 import myParol6, myParol6Config

SCENE_CMD_ADDR = ("127.0.0.1", 5006)  # mj_sim.py
GRIPPER_OPEN = 0


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--repo-id", required=True, help="e.g. fudayl/parol6_sim_stack")
    p.add_argument("--task", required=True, help='e.g. "put the green cube on the red cube"')
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--max-episode-s", type=float, default=60.0)
    p.add_argument("--host", default="127.0.0.1", help="controller address")
    p.add_argument("--sim", action="store_true", help="cameras from mj_sim.py, cube reset after each episode")
    p.add_argument("--stacked", action="store_true", help="sim reset starts with the cubes stacked")
    p.add_argument("--root", default=None, help="local dataset dir (default: HF_LEROBOT_HOME/<repo-id>)")
    p.add_argument("--resume", action="store_true", help="append episodes to an existing dataset")
    p.add_argument("--downscale", type=int, default=1, help="divide camera width/height by this (2: 720p -> 640x360)")
    p.add_argument("--crops", type=json.loads, default={}, help='per-camera crop, e.g. \'{"cam_2": [320, 0, 640, 480]}\'')
    p.add_argument("--downscales", type=json.loads, default={}, help='per-camera downscale, e.g. \'{"cam_1": 2}\'')
    return p.parse_args()


class Keys:
    def __init__(self):
        self.toggle = threading.Event()
        self.discard = threading.Event()
        self.quit = threading.Event()
        keyboard.Listener(on_press=self._on_press).start()

    def _on_press(self, key):
        if key == keyboard.Key.right:
            self.toggle.set()
        elif key == keyboard.Key.left:
            self.discard.set()
        elif key == keyboard.Key.esc:
            self.quit.set()

    def clear(self):
        self.toggle.clear()
        self.discard.clear()


def main():
    args = parse_args()
    keys = Keys()
    scene_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scene_sock.settimeout(1.0)

    robot = myParol6(myParol6Config(host=args.host, sim=args.sim, downscale=args.downscale, crops=args.crops, downscales=args.downscales))
    robot.connect()

    features = {
        **hw_to_dataset_features(robot.action_features, "action"),
        **hw_to_dataset_features(robot.observation_features, "observation"),
    }
    if args.resume:
        dataset = LeRobotDataset.resume(
            args.repo_id,
            root=args.root or HF_LEROBOT_HOME / args.repo_id,
            image_writer_threads=4 * len(robot.cameras),
            streaming_encoding=True,
        )
        mismatch = any(
            k not in dataset.features or dataset.features[k].get("names") != ft.get("names")
            for k, ft in features.items()
        )
        if mismatch or dataset.fps != args.fps:
            raise SystemExit("Existing dataset has different features/fps than this robot config; use a new --repo-id.")
        print(f"Appending to {args.repo_id} ({dataset.meta.total_episodes} episodes already)")
    else:
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            features=features,
            root=args.root,
            robot_type=robot.name,
            use_videos=True,
            image_writer_threads=4 * len(robot.cameras),
            # Encode during capture so save_episode() doesn't block ~25 s per episode
            streaming_encoding=True,
        )

    scene_poses = {"last": None}
    poses_file = Path(dataset.root) / "meta" / "sim_cube_poses.json"

    def reset_robot_and_scene():
        robot.send_cmd("0 -50 50 4 44 3")
        robot.send_cmd(f"gripper {GRIPPER_OPEN}")
        time.sleep(0.3)  # let the controller flag the move as busy
        while robot.get_status()["busy"]:
            time.sleep(0.05)
        if args.sim:
            scene_sock.sendto(b"reset stacked" if args.stacked else b"reset", SCENE_CMD_ADDR)
            try:
                v = [float(t) for t in scene_sock.recv(256).decode().split()[1:]]
                scene_poses["last"] = {"red": v[:4], "green": v[4:]}  # x y z yaw
            except (socket.timeout, ValueError):
                scene_poses["last"] = None
                print("  (mj_sim did not report cube poses; replay will estimate them from the image)")

    def store_scene_poses(episode_index):
        if scene_poses["last"] is None:
            return
        all_poses = json.loads(poses_file.read_text()) if poses_file.exists() else {}
        all_poses[str(episode_index)] = scene_poses["last"]
        poses_file.write_text(json.dumps(all_poses, indent=1))

    saved = 0
    try:
        reset_robot_and_scene()
        while saved < args.episodes and not keys.quit.is_set():
            keys.clear()
            print(f"\n[{saved}/{args.episodes}] Ready. Press -> to start recording (Esc to finish).")
            while not keys.toggle.is_set() and not keys.quit.is_set():
                time.sleep(0.02)
            if keys.quit.is_set():
                break
            keys.clear()

            print("  recording...  -> save | <- discard")
            period = 1.0 / args.fps
            t_end = time.perf_counter() + args.max_episode_s
            next_tick = time.perf_counter()
            n = 0
            first_state = None
            moved = False
            while not (keys.toggle.is_set() or keys.discard.is_set() or keys.quit.is_set()):
                if time.perf_counter() > t_end:
                    print("  max episode length reached")
                    break
                obs = robot.get_observation()
                action = robot.get_commanded_action()
                state = [obs[k] for k in robot.action_features]
                if first_state is None:
                    first_state = state
                elif max(abs(a - b) for a, b in zip(state, first_state)) > 0.5:
                    moved = True
                frame = {
                    **build_dataset_frame(dataset.features, obs, "observation"),
                    **build_dataset_frame(dataset.features, action, "action"),
                    "task": args.task,
                }
                dataset.add_frame(frame)
                n += 1

                next_tick += period
                sleep = next_tick - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.perf_counter()  # fell behind; don't try to burst-catch-up

            if keys.discard.is_set() or n == 0:
                dataset.clear_episode_buffer()
                print("  discarded")
            elif not moved:
                dataset.clear_episode_buffer()
                print(f"  DISCARDED: joints never moved. Is the teleop driving the controller at {args.host}?")
            else:
                print(f"  saving {n} frames ({n / args.fps:.1f}s)...")
                dataset.save_episode()
                store_scene_poses(dataset.meta.total_episodes - 1)
                saved += 1

            print("  returning to ready (release the teleop clutch)")
            reset_robot_and_scene()
    finally:
        print(f"\nFinalizing dataset with {saved} episodes -> {dataset.root}")
        dataset.finalize()
        robot.disconnect()


if __name__ == "__main__":
    main()
