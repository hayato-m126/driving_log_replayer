#!/usr/bin/env python3

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

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("talker")
        self.declare_parameter("prefix_msg", "default pre")
        self._prefix_msg = self.get_parameter("prefix_msg").get_parameter_value().string_value
        self.declare_parameter("postfix_msg", "default post")
        self._postfix_msg = self.get_parameter("postfix_msg").get_parameter_value().string_value

        self.i = 0
        self.pub = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        msg = String()
        msg.data = f"{self._prefix_msg}-{self.i}-{self._postfix_msg}"
        self.i += 1
        self.pub.publish(msg)


def main(args=None) -> None:
    # Run standalone
    rclpy.init(args=args)
    try:
        talker = Talker()
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
