#!/bin/bash

# 获取脚本所在的目录并切换到那里
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cd "$SCRIPT_DIR" || exit 1

# 运行 Python 脚本
python3 oculus_reader/start_franka_vr.py