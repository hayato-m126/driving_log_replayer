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
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def launch_bag_player(context: LaunchContext) -> IncludeLaunchDescription:
    scenario_path = Path(context.launch_configurations["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    params = yaml_obj["/**"]["ros__parameters"]
    input_bag_path = Path(params["dataset_path"], "input_bag")
    if not input_bag_path.is_absolute():
        input_bag_path = scenario_path.parent.joinpath(input_bag_path)

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            input_bag_path.as_posix(),
            "--delay",
            LaunchConfiguration("bag_play_delay"),
            "--rate",
            LaunchConfiguration("bag_play_rate"),
            "--clock",
            "200",
        ],
        output="screen",
    )
    return [
        bag_player,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=bag_player,
                on_exit=[
                    LogInfo(msg="player exit tearing down entire system."),
                    EmitEvent(event=Shutdown()),
                ],
            ),
        ),
    ]


def launch_autoware(context: LaunchContext) -> IncludeLaunchDescription:
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    scenario_path = Path(context.launch_configurations["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    params = yaml_obj["/**"]["ros__parameters"]
    map_path = Path(params["map_path"])
    if not map_path.is_absolute():
        map_path = scenario_path.parent.joinpath(map_path)
    launch_args = {
        "map_path": map_path.as_posix(),
        "vehicle_model": params["vehicle_model"],
        "sensor_model": params["sensor_model"],
        "vehicle_id": params["vehicle_id"],
    }
    return [
        LogInfo(msg=f"{map_path=}"),
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
    launch_names_str = context.launch_configurations["evaluations"]
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
                    "scenario_file": context.launch_configurations["scenario_path"],
                }.items(),
            ),
        )
    return multi_launch


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_play_rate",
                description="bag play rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "bag_play_delay",
                description="bag play delay",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "scenario_path",
                description="scenario file",
                default_value="/home/hyt/dlr2/dlr_sample.yaml",
            ),
            DeclareLaunchArgument(
                "evaluations",
                description="launch evaluation(s)",
                default_value="perception",
            ),
            OpaqueFunction(function=launch_autoware),
            OpaqueFunction(function=launch_bag_player),
            # OpaqueFunction(function=launch_evaluators),
        ],
    )
