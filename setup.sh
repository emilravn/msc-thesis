#!/bin/bash
set -e

# vcs import < src/ros2.repos src (uncomment if need for )
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y