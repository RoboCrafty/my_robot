"""Cut a local LeRobot v3 dataset into one side-by-side mp4 per episode for quick review.

uv run review_episodes.py RoboCrafty/parol6_stack_real_4 [--speed 2] [--out review/<name>]
Then open the folder in Finder and use Quick Look (space + arrow keys) to step through episodes.
"""
import argparse
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pandas as pd

from lerobot.utils.constants import HF_LEROBOT_HOME


def main():
    p = argparse.ArgumentParser()
    p.add_argument("repo_id")
    p.add_argument("--speed", type=float, default=2.0)
    p.add_argument("--width", type=int, default=640, help="width of each camera tile")
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--jobs", type=int, default=4)
    args = p.parse_args()

    root = HF_LEROBOT_HOME / args.repo_id
    out = args.out or Path("review") / args.repo_id.split("/")[-1]
    out.mkdir(parents=True, exist_ok=True)

    eps = pd.concat(pd.read_parquet(f) for f in sorted((root / "meta/episodes").glob("*/*.parquet")))
    cams = sorted({c.split("/")[1] for c in eps.columns if c.startswith("videos/") and c.endswith("/file_index")})

    def cut(row) -> str:
        ep = int(row["episode_index"])
        dst = out / f"ep_{ep:03d}.mp4"
        if dst.exists():
            return f"skip {dst.name}"
        cmd = ["ffmpeg", "-loglevel", "error", "-y"]
        for cam in cams:
            k = f"videos/{cam}"
            src = root / "videos" / cam / f"chunk-{int(row[k + '/chunk_index']):03d}" / f"file-{int(row[k + '/file_index']):03d}.mp4"
            start, end = row[k + "/from_timestamp"], row[k + "/to_timestamp"]
            cmd += ["-ss", f"{start:.3f}", "-t", f"{end - start:.3f}", "-i", str(src)]
        scaled = "".join(f"[{i}:v]scale={args.width}:-2[v{i}];" for i in range(len(cams)))
        stacked = "".join(f"[v{i}]" for i in range(len(cams)))
        stack = f"hstack=inputs={len(cams)}" if len(cams) > 1 else "null"
        cmd += ["-filter_complex", f"{scaled}{stacked}{stack},setpts=PTS/{args.speed}",
                "-an", "-c:v", "h264_videotoolbox", "-b:v", "4M", str(dst)]
        subprocess.run(cmd, check=True)
        return f"wrote {dst.name} ({int(row['length'])} frames)"

    with ThreadPoolExecutor(args.jobs) as pool:
        for msg in pool.map(cut, [r for _, r in eps.iterrows()]):
            print(msg)
    print(f"\n{len(eps)} episodes in {out.resolve()}  (cams left->right: {', '.join(cams)})")


if __name__ == "__main__":
    main()
