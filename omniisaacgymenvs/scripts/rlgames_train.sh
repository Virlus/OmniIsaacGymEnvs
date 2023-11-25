#!/bin/bash

srun -p mozi --gres=gpu:8 ~/ov/pkg/isaac_sim-2023.1.0-hotfix.1/python.sh -m torch.distributed.launch --use_env scripts/rlgames_train.py task=$1 headless=True