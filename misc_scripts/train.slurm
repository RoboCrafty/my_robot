#!/bin/bash
#SBATCH --job-name=lerobot_train         # Name of your job
#SBATCH --partition=h200                # Use the A6000 GPU partition
#SBATCH --nodes=1                        # Request 1 compute node
#SBATCH --ntasks=1                       # Run 1 task
#SBATCH --cpus-per-task=96               # pyav decoding of 3 cams is the bottleneck; each worker uses ~1.5 cores
#SBATCH --mem=256G                       # prefetched batches live in shared memory and count against this
#SBATCH --gres=gpu:1                     # Request 1 GPU
#SBATCH --time=24:00:00                  # Max runtime (e.g., 24 hours)
#SBATCH --output=outputs/slurm-%j.out    # Where to save standard output (%j is Job ID)
#SBATCH --error=outputs/slurm-%j.err     # Where to save error logs

# 1. Clear any weird SSL variables that might break VSCode/SLURM interaction
unset ${!SSL_*}

# 2. Navigate to your working directory (Change this to your actual path!)
cd /sharchome/ft-rpd-ced-inw-de/fuzail

# 3. 'module' isn't defined in batch shells; the .venv already ships torch+CUDA, so no modules needed.
# source /etc/profile.d/z00-lmod.sh && module load ...

## ACT 
# 4. Run your command!
# uv run lerobot-train \
#     --policy.type=act \
#     --policy.device=cuda \
#     --policy.push_to_hub=true \
#     --dataset.repo_id=RoboCrafty/parol6_sim_stack_real_1 \
#     --output_dir=outputs/act_real_1_2 \
#     --steps=80000 \
#     --save_freq=5000 \
#     --batch_size=16 \
#     --num_workers=8 \
#     --policy.repo_id=RoboCrafty/act_parol6_real_1_2 --wandb.enable=true

uv run lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.device=cuda \
  --policy.push_to_hub=true --policy.repo_id=RoboCrafty/smolvla_parol6_real_1_2 \
  --dataset.repo_id=RoboCrafty/parol6_sim_stack_real_1 \
  --output_dir=outputs/smolvla_real_1_2 \
  --steps=20000 --save_freq=2500 --batch_size=256 --num_workers=64 --prefetch_factor=2 \
  --policy.optimizer_lr=2e-4 \
  --wandb.enable=true --rename_map='{"observation.images.cam_1": "observation.images.camera1", "observation.images.cam_2": "observation.images.camera2"}'
