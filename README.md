# DLR2 for ROS 2 Autoware.Universe

DLR2 is a ROS package that evaluates the functionality of Autoware.Universe

## Requirements

- ROS 2 humble
- [Python 3.10](https://www.python.org/)

## Installation

### How to install dlr2

Use colcon build

```shell
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to dlr2
```

## Usage

refer [document](https://tier4.github.io/dlr2)

## (For Developer) Release Process

This package uses `catkin_pkg` to manage releases.

Refer [this page](https://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage)

### Release command

Can only be executed by users with repository maintainer privileges

```shell
# create change log
catkin_generate_changelog
# edit CHANGELOG.rst
# update package version in pyproject.toml
# edit ReleaseNotes.md
# commit and create pull request
# merge pull request
catkin_prepare_release
# When you type the command, it automatically updates CHANGELOG.rst and creates a git tag
git checkout main
git merge develop
git push origin main
```
