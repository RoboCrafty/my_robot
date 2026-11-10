# SLURM / GPU Cheat Sheet

## Submit & manage jobs
```bash
sbatch train.slurm                       # submit
squeue -u $USER                          # my jobs
squeue -u $USER --start                  # estimated start time of pending jobs
scancel <JOBID>                          # cancel one job
scancel -u $USER                         # cancel all my jobs
scontrol show job <JOBID>                # full details (node, CPUs, mem, GPU, reason)
sacct -j <JOBID> --format=JobID,State,Elapsed,MaxRSS,AllocCPUS,ReqMem   # finished job stats
seff <JOBID>                             # CPU/mem efficiency summary (if installed)
```

## Cluster / partition info
```bash
sinfo -p h200 -o "%N %c %m %G %t"        # nodes, CPUs, mem, GPUs, state
sinfo -s                                 # all partitions summary
scontrol show node slurm-node05          # node details (allocated CPUs/GPUs)
```

## Logs
```bash
tail -f outputs/slurm-<JOBID>.err        # follow live log (lerobot logs to stderr)
grep "step:" outputs/slurm-<JOBID>.err | tail   # just training progress lines
```

## Attach to a RUNNING job (monitoring)
`--overlap` lets you share the job's allocation without stopping it.
```bash
srun --jobid=<JOBID> --overlap --pty bash            # interactive shell on the job's node
srun --jobid=<JOBID> --overlap --pty nvitop          # live htop-style GPU monitor (q to quit)
srun --jobid=<JOBID> --overlap nvitop -1             # one-shot GPU snapshot
srun --jobid=<JOBID> --overlap top -bn1 -u $USER | head -30   # CPU usage of dataloader workers
srun --jobid=<JOBID> --overlap nproc                 # how many CPUs the job actually has
```
Inside the `--pty bash` shell: `nvitop`, `htop -u $USER`.

## Interactive GPU session (debugging / benchmarking)
```bash
srun -p h200 --gres=gpu:1 --cpus-per-task=32 --mem=128G --time=02:00:00 --pty bash
# then: cd ~/fuzail && uv run lerobot-train ... --steps=500
```

## Diagnosing low GPU utilization
In the lerobot log line `updt_s:… data_s:… smp/s:…`:
- `data_s` high -> data loading is the bottleneck (more CPUs / workers, faster decoder).
- `updt_s` high but GPU util low -> main process is CPU-starved (too many workers for the
  allocated cores) or tiny kernels; enable AMP, bigger batch, `--policy.compile_model=true`.
- `top` shows all cores at ~100% and load average >> `nproc` -> request more `--cpus-per-task`.
- Rule of thumb: `--num_workers` <= `--cpus-per-task` - 4.

## Useful lerobot flags for throughput
```bash
--num_workers=24 --batch_size=128      # scale lr if you change batch size a lot
--policy.use_amp=true                  # mixed precision
--policy.compile_model=true            # torch.compile (slow first steps)
--dataset.video_backend=pyav           # CPU decode (current fallback; torchcodec needs FFmpeg libs)
```
