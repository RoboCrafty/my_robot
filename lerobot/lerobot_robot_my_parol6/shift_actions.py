"""Copy a LeRobot v3 dataset with each episode's actions shifted k frames into the future: action[t] := action[t+k].

Our recorded action (controller `tgt`) equals the measured state at the same frame, so a policy learns "stay where you
are" as its next action. Shifting makes action[t] the pose the arm reached k frames later, like a leader arm leading.

uv run shift_actions.py RoboCrafty/parol6_stack_real_4 RoboCrafty/parol6_stack_real_4_shift1 [--k 1]
"""
import argparse
import shutil

import numpy as np
import pandas as pd
from huggingface_hub import snapshot_download

from lerobot.utils.constants import HF_LEROBOT_HOME


def shift_episode(actions: np.ndarray, k: int) -> np.ndarray:
    return np.concatenate([actions[k:], np.repeat(actions[-1:], min(k, len(actions)), axis=0)])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("src_repo_id")
    p.add_argument("dst_repo_id")
    p.add_argument("--k", type=int, default=1)
    args = p.parse_args()

    src = HF_LEROBOT_HOME / args.src_repo_id
    # always sync: a cache from lerobot-doctor etc. can have meta+data but no videos
    try:
        snapshot_download(args.src_repo_id, repo_type="dataset", local_dir=src)
    except Exception as e:
        print(f"hub sync skipped ({type(e).__name__}); using local copy")
    if not any((src / "videos").rglob("*.mp4")):
        raise SystemExit(f"no videos under {src}/videos -- source dataset incomplete")
    dst = HF_LEROBOT_HOME / args.dst_repo_id
    if dst.exists():
        raise SystemExit(f"{dst} already exists")
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns(".cache"))

    for f in sorted((dst / "data").glob("*/*.parquet")):
        df = pd.read_parquet(f)
        new = np.stack(df["action"].values).astype(np.float32)
        # episodes never span files in v3, so shifting per episode within a file is safe
        for _, idx in df.groupby("episode_index").indices.items():
            order = idx[np.argsort(df["frame_index"].values[idx])]
            new[order] = shift_episode(new[order], args.k)
        df["action"] = list(new)
        df.to_parquet(f, index=False)
        print(f"shifted {f.relative_to(dst)}")

    a = pd.concat(pd.read_parquet(f) for f in sorted((dst / "data").glob("*/*.parquet")))
    s, a = np.stack(a["observation.state"].values), np.stack(a["action"].values)
    np.set_printoptions(precision=3, suppress=True)
    print(f"\nmean |action - state| per dim after shift k={args.k}: {np.abs(a - s).mean(0)}")
    print(f"wrote {dst}  (normalization stats kept: only the last {args.k} frame(s) per episode change)")


if __name__ == "__main__":
    main()
