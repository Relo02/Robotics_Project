#!/usr/bin/env python
import bagpy
from bagpy import bagreader

bag_path = "project.bag"
topic_name = "/swiftnav/front/gps_pose"  # e.g., "/gps/fix"

# Open the bag and extract the first message
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, timestamp in bag.read_messages(topics=[topic_name]):
        print(f"First message on {topic}:")
        print(msg)
        break  # Exit after the first message