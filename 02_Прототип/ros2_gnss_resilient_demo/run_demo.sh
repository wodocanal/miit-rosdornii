#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # Do not let a previously sourced colcon workspace redirect Jazzy's setup.
  unset COLCON_CURRENT_PREFIX
  source /opt/ros/jazzy/setup.bash
fi

if [ ! -f install/setup.bash ]; then
  colcon build --packages-select gnss_resilient_demo --symlink-install
fi
source install/setup.bash

if [ "${1:-}" = "--headless" ]; then
  shift
  ros2 run gnss_resilient_demo demo_headless "$@"
else
  ros2 launch gnss_resilient_demo demo.launch.py
fi
