# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `driving_log_replayer`.

## Requirements

- CPU amd64
- Ubuntu 22.04
- ROS humble
- Python 3.10
- NVIDIA GPU (required if running perception)
- [pipx](https://pypa.github.io/pipx/)
- [zstd](https://github.com/facebook/zstd)
  - sudo apt install zstd

## How to build

1. Navigate to the Autoware workspace:

   ```shell
   cd autoware
   ```

2. Add dependency packages:

   ```shell
   nano simulator.repos
   # add repositories below.
   ```

   ```shell
     simulator/perception_eval:
       type: git
       url: https://github.com/tier4/autoware_perception_evaluation.git
       version: main
     simulator/driving_log_replayer:
       type: git
       url: https://github.com/tier4/driving_log_replayer.git
       version: main
     simulator/vendor/ros2_numpy:
       type: git
       url: https://github.com/Box-Robotics/ros2_numpy.git
       version: humble
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
   ```

3. Import Simulator dependencies:

   ```shell
   vcs import src < simulator.repos
   ```

4. Update rosdep:

   ```shell
   rosdep update
   ```

5. Install dependent ROS packages:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. Build the workspace:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

7. Install cli to run driving_log_replayer:

   ```shell
   pipx install git+https://github.com/tier4/driving_log_replayer.git
   ```

8. Install shell completion scripts

   ```shell
   # bash
   _DLR_COMPLETE=bash_source dlr > $HOME/.dlr-complete.bash
   _DLR_COMPLETE=bash_source dlr > $HOME/.dlr-analyzer-complete.bash

   echo "source $HOME/.dlr-complete.bash" >> ~/.bashrc
   echo "source $HOME/.dlr-analyzer-complete.bash" >> ~/.bashrc
   ```

   ```shell
   # fish
   _DLR_COMPLETE=fish_source dlr > $HOME/.config/fish/completions/dlr.fish
   _DLR_ANALYZER_COMPLETE=fish_source dlr-analyzer > $HOME/.config/fish/completions/dlr-analyzer.fish
   ```
