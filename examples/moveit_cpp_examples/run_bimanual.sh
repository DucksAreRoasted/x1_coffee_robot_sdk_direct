#!/usr/bin/env bash
set -euo pipefail
execute=false
if [[ "${1:-}" == "--execute" ]]; then
  execute=true
fi
ros2 launch x1_coffee_moveit_cpp_examples coffee_moveit_cpp_demo.launch.py \
  mode:=both execute:="${execute}"
