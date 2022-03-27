# Copyright 2021 Tier IV, Inc. All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # use_sim_time
    set_use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    occlusion_spot_generator_param_path = os.path.join(
        get_package_share_directory("occlusion_spot_generator"),
        "config",
        "occlusion_spot_generator.param.yaml",
    )
    # occlusion_spot_generator
    load_occlusion_spot_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("occlusion_spot_generator"),
                "/launch/occlusion_spot_generator.launch.py",
            ]
        ),
        launch_arguments={
            "occlusion_spot_generator_param_file": [occlusion_spot_generator_param_path]
        }.items(),
    )

    return [
        set_use_sim_time,
        load_occlusion_spot_generator,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
