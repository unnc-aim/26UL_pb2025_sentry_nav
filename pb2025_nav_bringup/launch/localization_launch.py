# Copyright 2025 Lihan Chen
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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_localization(context):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_amcl = IfCondition(LaunchConfiguration("use_amcl")).evaluate(context)
    use_composition = IfCondition(LaunchConfiguration("use_composition")).evaluate(
        context
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    lifecycle_nodes = ["map_server", "amcl"] if use_amcl else ["map_server"]
    lifecycle_params = [
        {
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": lifecycle_nodes,
        }
    ]
    localizer_params = [configured_params]
    if use_amcl:
        package, executable, plugin, name = (
            "nav2_amcl",
            "amcl",
            "nav2_amcl::AmclNode",
            "amcl",
        )
        localizer_params.append({"set_initial_pose": False})
    else:
        package = name = "small_gicp_relocalization"
        executable = "small_gicp_relocalization_node"
        plugin = "small_gicp_relocalization::SmallGicpRelocalizationNode"
        localizer_params.append(
            {
                "prior_pcd_file": prior_pcd_file,
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            }
        )

    lio_params = [configured_params, {"prior_pcd.prior_pcd_map_path": prior_pcd_file}]
    if use_amcl:
        lio_params.append({"prior_pcd.enable": False, "pcd_save.pcd_save_en": False})
    actions = [
        Node(
            condition=UnlessCondition(LaunchConfiguration("use_external_odometry")),
            package="point_lio",
            executable="pointlio_mapping",
            name="point_lio",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=lio_params,
            arguments=["--ros-args", "--log-level", log_level],
        )
    ]
    if use_composition:
        actions.append(
            LoadComposableNodes(
                target_container=(
                    namespace,
                    "/",
                    LaunchConfiguration("container_name"),
                ),
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_map_server",
                        plugin="nav2_map_server::MapServer",
                        name="map_server",
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package=package,
                        plugin=plugin,
                        name=name,
                        parameters=localizer_params,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_localization",
                        parameters=lifecycle_params,
                    ),
                ],
            )
        )
    else:
        actions.extend(
            [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=["--ros-args", "--log-level", log_level],
                ),
                Node(
                    package=package,
                    executable=executable,
                    name=name,
                    output="screen",
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=localizer_params,
                    arguments=["--ros-args", "--log-level", log_level],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_localization",
                    output="screen",
                    parameters=lifecycle_params,
                    arguments=["--ros-args", "--log-level", log_level],
                ),
            ]
        )
    if use_amcl:
        actions.append(
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")],
                arguments=["--ros-args", "--log-level", log_level],
            )
        )
    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("map", description="Saved occupancy map YAML"),
            DeclareLaunchArgument("use_amcl", default_value="False"),
            DeclareLaunchArgument("use_external_odometry", default_value="False"),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("prior_pcd_file", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "reality", "nav2_params.yaml"
                ),
            ),
            DeclareLaunchArgument("autostart", default_value="True"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("container_name", default_value="nav2_container"),
            DeclareLaunchArgument("use_respawn", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.7"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.5"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            OpaqueFunction(function=launch_localization),
        ]
    )
