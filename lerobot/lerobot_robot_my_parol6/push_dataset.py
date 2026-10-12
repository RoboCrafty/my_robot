import sys

from huggingface_hub import HfApi

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import HF_LEROBOT_HOME

REPO_ID = sys.argv[1] if len(sys.argv) > 1 else "RoboCrafty/parol6_stack_real_2"

if __name__ == "__main__":
    HfApi().create_repo(REPO_ID, repo_type="dataset", private=True, exist_ok=True)
    # explicit root: without it LeRobot treats the dataset as a Hub download and looks it up remotely first
    LeRobotDataset(REPO_ID, root=HF_LEROBOT_HOME / REPO_ID).push_to_hub(private=True)
    