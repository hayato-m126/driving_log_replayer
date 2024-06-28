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
import datetime
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
from launch_ros.actions import Node
import yaml

from dlr2.shutdown_once import ShutdownOnce


def launch_bag_player(context: LaunchContext) -> list:
    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            context.launch_configurations["input_bag"],
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
                    LogInfo(msg="Player exited; tearing down entire system."),
                    # EmitEvent(event=ShutdownOnce()), # not working
                    EmitEvent(event=Shutdown()),
                ],
            ),
        ),
    ]


def launch_bag_recorder(context: LaunchContext) -> list:
    output_path = Path(context.launch_configurations["output_path"], "result_bag")

    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-s",
                "mcap",
                "-o",
                output_path.as_posix(),
                # "--qos-profile-overrides-path",
                # Path(
                #     get_package_share_directory("driving_log_replayer"),
                #     "config",
                #     record_config_name,
                # ).as_posix(),
                # "-e",
                # allowlist,
                "-a",
                "--use-sim-time",
            ],
        ),
    ]


def launch_autoware(context: LaunchContext) -> list:
    autoware_launch_file = Path(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    scenario_path = Path(context.launch_configurations["scenario_path"])
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
    params = {
        "use_sim_time": True,
        "scenario_path": context.launch_configurations["scenario_path"],
        "result_json_path": Path(
            context.launch_configurations["output_path"],
            "result.jsonl",
        ).as_posix(),
        "t4_dataset_path": context.launch_configurations["dataset_path"],
        "result_archive_path": Path(
            context.launch_configurations["output_path"],
            "result_archive",
        ).as_posix(),
    }

    multi_launch = []
    launch_names_str = context.launch_configurations["evaluations"]
    launch_names_list: list = literal_eval(launch_names_str)
    for node_name in launch_names_list:
        multi_launch.append(
            Node(
                package="dlr2",
                namespace="/dlr2",
                executable=node_name + "_evaluator_node.py",
                output="screen",
                name=node_name + "_evaluator",
                parameters=[context.launch_configurations["scenario_path"], params],
                on_exit=ShutdownOnce(),
            ),
        )
    return multi_launch


def update_resource_path(context: LaunchContext) -> list:
    scenario_path = Path(context.launch_configurations["scenario_path"])
    with scenario_path.open() as scenario_file:
        yaml_obj = yaml.safe_load(scenario_file)
    params = yaml_obj["/**"]["ros__parameters"]
    map_path = Path(params["map_path"])
    if not map_path.is_absolute():
        map_path = scenario_path.parent.joinpath(map_path)
    dataset_path = Path(params["dataset_path"])
    if not dataset_path.is_absolute():
        dataset_path = scenario_path.parent.joinpath(dataset_path)
    context.launch_configurations["map_path"] = map_path.as_posix()
    context.launch_configurations["dataset_path"] = dataset_path.as_posix()
    # add configurations
    context.launch_configurations["input_bag"] = dataset_path.joinpath("input_bag").as_posix()
    return [
        LogInfo(msg=f"{map_path=}, {dataset_path=}"),
    ]


def create_output_path(context: LaunchContext) -> list:
    output_dir_by_time = Path(
        context.launch_configurations["output_path"],
        datetime.datetime.now().strftime("%Y-%m%d-%H%M%S"),  # noqa
    )
    output_dir_by_time.mkdir()
    context.launch_configurations["output_path"] = output_dir_by_time.as_posix()
    return [LogInfo(msg=f"{output_dir_by_time=}")]


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
                default_value="[]",  # default launch no evaluator node
            ),
            DeclareLaunchArgument(
                "output_path",
                description="Directory to output result.jsonl, rosbag, and pickle",
                default_value="/home/hyt/dlr2/output",
            ),
            OpaqueFunction(function=update_resource_path),
            OpaqueFunction(function=create_output_path),
            OpaqueFunction(function=launch_autoware),
            OpaqueFunction(function=launch_bag_player),
            OpaqueFunction(function=launch_bag_recorder),
            OpaqueFunction(function=launch_evaluators),
        ],
    )
