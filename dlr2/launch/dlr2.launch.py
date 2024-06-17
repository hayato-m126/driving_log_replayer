# Copyright (c) 2024 TIER IV.inc
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

from ast import literal_eval
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def launch_autoware(context: LaunchContext) -> IncludeLaunchDescription:
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    scenario_path = Path(context.launch_configurations["dlr_scenario_file"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    params = yaml_obj["/**"]["ros__parameters"]
    launch_args = {
        "map_path": params["map_path"],
        "vehicle_model": params["vehicle_model"],
        "sensor_model": params["sensor_model"],
        "vehicle_id": params["vehicle_id"],
    }
    return [
        GroupAction(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        autoware_launch_file.as_posix(),
                    ),
                    launch_arguments=launch_args.items(),
                    # condition=IfCondition(LaunchConfiguration("with_autoware")),
                ),
            ],
            scoped=False,
            forwarding=True,
        ),
    ]


def launch_evaluators(context: LaunchContext) -> list:
    launch_file_dir = get_package_share_directory("dlr2") + "/launch/"

    multi_launch = []
    launch_names_str = context.launch_configurations["dlr_launch_evaluations"]
    launch_names_list: list = literal_eval(launch_names_str)
    for launch_name in launch_names_list:
        multi_launch.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    [
                        launch_file_dir,
                        launch_name,
                        ".launch.py",
                    ],  # 文字列結合
                ),
                launch_arguments={
                    "scenario_file": context.launch_configurations["dlr_scenario_file"],
                }.items(),
            ),
        )
    return multi_launch


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dlr_scenario_file",
                description="scenario file",
                default_value="/home/hyt/dlr_sample.yaml",
            ),
            DeclareLaunchArgument(
                "dlr_launch_evaluations",
                description="launch evaluation(s)",
                default_value="perception",
            ),
            OpaqueFunction(function=launch_autoware),
            # OpaqueFunction(function=launch_evaluators),
        ],
    )
