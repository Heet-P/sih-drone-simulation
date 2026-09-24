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
Launch the S500 quad on the post-disaster world (disaster.sdf).
Same as `s500_quad_runway.launch.py world:=disaster`; all its other
arguments (rviz, use_gz_tf, ...) pass through.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate a launch description for the S500 quad in disaster.sdf."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f'{Path(pkg_project_bringup) / "launch" / "s500_quad_runway.launch.py"}'
                ),
                launch_arguments={"world": "disaster"}.items(),
            ),
        ]
    )
