#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("gamepad")
@dataclass
class GamepadTeleopConfig(TeleoperatorConfig):
    use_gripper: bool = True
    wz_step_size: float = 0.12
    # Gripper: use trigger axes by default (LT open, RT close on Xbox/SDL).
    # Set gripper_*_button to a pygame button index to use a digital button instead.
    gripper_open_button: int | None = None
    gripper_close_button: int | None = None
    gripper_open_trigger_axis: int = 2
    gripper_close_trigger_axis: int = 5
