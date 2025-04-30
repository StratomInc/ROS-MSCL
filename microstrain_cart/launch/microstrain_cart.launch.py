# Copyright (c) 2025 Stratom, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of Stratom, Inc.
# You may not copy, modify, distribute, or disclose this software or its contents
# without the express written permission of Stratom, Inc.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def setup_launch(context, *args, **kwargs):
    """Help launch function."""
    launch = []

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("microstrain_cart"),
                    "urdf",
                    "microstrain_cart.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    launch.append(node_robot_state_publisher)

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    # )
    # launch.append(joint_state_broadcaster_spawner)

    # INS launch
    _MICROSTRAIN_LAUNCH_FILE = os.path.join(
        get_package_share_directory("microstrain_inertial_driver"),
        "launch",
        "microstrain_launch.py",
    )
    _GQ7_PARAMS_FILE = os.path.join(
        get_package_share_directory("microstrain_cart"), "config", "gq7.yaml"
    )

    microstrain = GroupAction(
        actions=[
            SetRemap(src="/gnss_1/llh_position", dst="/gnss1/fix"),
            SetRemap(src="/gnss_2/llh_position", dst="/gnss2/fix"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
                launch_arguments={
                    "configure": "true",
                    "activate": "true",
                    "params_file": _GQ7_PARAMS_FILE,
                    "namespace": "/",
                }.items(),
            ),
        ]
    )

    launch.append(microstrain)

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("microstrain_cart"),
                "config",
                "microstrain_cart.rviz",
            ),
            "--ros-args",
            "--log-level",
            "fatal",
        ],
    )

    launch.append(rviz_launch)

    return launch


def generate_launch_description():
    """Generate launch description function."""
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=setup_launch))
    return ld
