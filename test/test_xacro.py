# Copyright 2024 Husarion sp. z o.o.
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

import itertools
import os

import xacro
from ament_index_python.packages import get_package_share_directory


def test_robot_description_parsing():
    robot_model_values = ["robot", "robot_xl"]
    mecanum_values = ["True", "False"]

    all_combinations = list(
        itertools.product(
            robot_model_values,
            mecanum_values,
        )
    )

    for combination in all_combinations:
        robot_model, mecanum = combination

        mappings = {
            "mecanum": mecanum,
        }
        robot_description = get_package_share_directory("robot_description")
        xacro_path = os.path.join(robot_description, "urdf", f"{robot_model}.urdf.xacro")
        try:
            xacro.process_file(xacro_path, mappings=mappings)
        except xacro.XacroException as e:
            assert (
                False
            ), f"xacro parsing failed: {str(e)} for robot_model: {robot_model}, mecanum: {mecanum}"
