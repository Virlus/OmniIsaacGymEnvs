#!/bin/bash

# srun -p mozi --gres=gpu:4 ~/ov/pkg/isaac_sim-2023.1.0-hotfix.1/python.sh -m torch.distributed.launch --use_env scripts/rlgames_train.py task=$1 headless=True
srun -p mozi --gres=gpu:1 ~/ov/pkg/isaac_sim-2023.1.0-hotfix.1/python.sh scripts/rlgames_train.py task=$1 experiment=$2 headless=True 