#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
moveit_controllers=(left_group_controller right_group_controller waist_group_controller)
openarm_controllers=(left_forward_position_controller right_forward_position_controller waist_forward_position_controller)

if [[ "$mode" == "openarm" ]]; then
  ros2 control switch_controllers \
    --deactivate "${moveit_controllers[@]}" \
    --activate "${openarm_controllers[@]}" \
    --strict
elif [[ "$mode" == "moveit" ]]; then
  ros2 control switch_controllers \
    --deactivate "${openarm_controllers[@]}" \
    --activate "${moveit_controllers[@]}" \
    --strict
else
  echo "usage: $0 {openarm|moveit}" >&2
  exit 2
fi
