#! /usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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


#!/usr/bin/env python3
# MATPLOTLIB_GRID
import argparse
import csv
from datetime import datetime
import os
from pprint import pprint
import sqlite3

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

"""
reference: https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
"""


# Example usage #
topic_names = [
    "/vehicle/status/velocity_status",
    "/sensing/lidar/left/velodyne_packets",
    "/sensing/lidar/top/velodyne_packets",
    "/sensing/lidar/right/velodyne_packets",
    "/perception/object_recognition/detection/objects",
    "/perception/object_recognition/tracking/objects",
    "/perception/object_recognition/objects",
]


class BagFileParser:
    def __init__(self, bag_file, stamp_type, window_size):
        self.conn = sqlite3.connect(bag_file)
        self.stamp_type = stamp_type
        self.window_size = window_size
        self.cursor = self.conn.cursor()

        # Create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

        # Example usage
        self.topic_names = topic_names

        self.topics = [self.get_messages(t) for t in self.topic_names]
        self.stamp_and_rate = {}
        for n in self.topic_names:
            self.stamp_and_rate[n] = []

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        # Deserialize all and timestamp them
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]

    def get_topic_rate(self, stamp_list):
        if len(stamp_list) < 2:
            return 0.0

        window_size = min(len(stamp_list), self.window_size)
        start_time = stamp_list[-window_size]
        end_time = stamp_list[-1]
        time_diff = end_time - start_time

        return (window_size - 1) / time_diff

    def run(self):
        for topic, name in zip(self.topics, self.topic_names):
            stamp_list = []
            for msg in topic:
                stamp = (
                    msg[0] / 1000000000
                    if self.stamp_type == "real"
                    else msg[1].header.stamp.sec + msg[1].header.stamp.nanosec / 1000000000
                )
                stamp_list.append(stamp)
                if len(stamp_list) > self.window_size:
                    topic_rate = self.get_topic_rate(stamp_list)
                    self.stamp_and_rate[name].append([stamp, topic_rate])

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlabel("stamp[s]")
        ax.set_ylabel("topic rate[hz]")
        ax.minorticks_on()
        ax.grid(which="major", axis="x", color="black", alpha=0.8, linestyle="--", linewidth=1)
        ax.grid(which="major", axis="y", color="gray", alpha=0.8, linestyle="--", linewidth=1)
        ax.grid(which="minor", axis="y", color="gray", alpha=0.8, linestyle="--", linewidth=1)
        for name in self.topic_names:
            arr = np.array(self.stamp_and_rate[name])
            stamp = arr[:, 0]
            rate = arr[:, 1]
            ax.plot(stamp, rate, label=name)

        ax.legend()
        plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="report hz from rosbag.")
    parser.add_argument("bag_file", help="input bagfile")
    parser.add_argument(
        "--stamp_type",
        "-s",
        default="real",
        type=str,
        choices=["real", "ideal"],
        help="real or ideal",
    )
    parser.add_argument(
        "--window_size",
        "-w",
        default=10,
        type=int,
        help="this should be more than 2 and less than number of messages",
    )
    args = parser.parse_args()

    parser = BagFileParser(args.bag_file, args.stamp_type, args.window_size)
    parser.run()
