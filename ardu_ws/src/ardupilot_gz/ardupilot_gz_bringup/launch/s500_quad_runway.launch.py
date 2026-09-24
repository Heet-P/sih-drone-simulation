# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Launch the S500 quad (SIH project airframe) on the runway world, or on
another world in ardupilot_gz_gazebo/worlds via world:=<name> (e.g.
world:=disaster, see s500_quad_disaster.launch.py). The world file's
<world name> must equal its file name, since the ROS bridge topics are
/world/<name>/...
Derived from iris_runway.launch.py, includes robots/s500_quad.launch.py
instead of robots/iris.launch.py. Your original iris_runway.launch.py
is untouched, both remain usable.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the S500 quad."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_project_bringup,
                        "launch",
                        "robots",
                        "s500_quad.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={"world_name": LaunchConfiguration("world")}.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": [
                f'-v4 -s -r {Path(pkg_project_gazebo) / "worlds"}/',
                LaunchConfiguration("world"),
                ".sdf",
            ]
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        # Project GUI config: the default one starts the world paused
        # when the GUI connects, which stalls SITL so the drone never arms.
        launch_arguments={
            "gz_args": "-v4 -g --gui-config "
            f'{Path(pkg_project_bringup) / "config" / "gz_gui.config"}'
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    # Reuses iris.rviz: TF frame names (base_link, imu_link, rotor_0..3)
    # come from the SDF's own link names, unchanged from Iris, and are
    # not prefixed with the robot name, so the existing RViz config
    # should display this model without changes.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="runway",
                description="World in ardupilot_gz_gazebo/worlds (runway, disaster).",
            ),
            DeclareLaunchArgument(
                "use_gz_sim_server",
                default_value="true",
                description="Run the Gazebo server.",
            ),
            DeclareLaunchArgument(
                "use_gz_sim_gui",
                default_value="true",
                description="Run the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description="Spawn the robot and start SITL+ROS.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
            robot,
            rviz,
        ]
    )
