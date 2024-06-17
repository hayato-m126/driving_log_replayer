# Copyright (c) 2023 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

import dlr2.launch_common as cmn

RECORD_TOPIC_REGEX = """^/clock$\
|^/tf$\
|^/sensing/lidar/concatenated/pointcloud$\
|^/perception/object_recognition/detection/objects$\
|^/perception/object_recognition/tracking/objects$\
|^/perception/object_recognition/objects$\
|^/sensing/camera/.*\
|^/driving_log_replayer/.*\
"""


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = cmn.get_launch_arguments()
    launch_arguments.append(DeclareLaunchArgument("sensing", default_value="false"))
    autoware_launch = cmn.get_autoware_launch(
        sensing=LaunchConfiguration("sensing"),
        localization="false",
        perception_mode="camera_lidar_fusion",
    )
    rviz_node = cmn.get_rviz("perception.rviz")
    evaluator_node = cmn.get_evaluator_node("perception_2d")

    player_normal = cmn.get_player(
        condition=UnlessCondition(LaunchConfiguration("sensing")),
    )

    player_remap = cmn.get_player(
        additional_argument=[
            "--remap",
            "/sensing/lidar/concatenated/pointcloud:=/driving_log_replayer/concatenated/pointcloud",
        ],
        condition=IfCondition(LaunchConfiguration("sensing")),
    )

    recorder, recorder_override = cmn.get_regex_recorders(
        "perception.qos.yaml",
        RECORD_TOPIC_REGEX,
    )

    return launch.LaunchDescription(
        [
            *launch_arguments,
            rviz_node,
            autoware_launch,
            evaluator_node,
            player_normal,
            player_remap,
            recorder,
            recorder_override,
        ],
    )