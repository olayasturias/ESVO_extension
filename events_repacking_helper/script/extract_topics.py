#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rosbag2_py
import os
import sys
import argparse

class ExtractTopics(Node):
    def __init__(self):
        super().__init__('extract_topics')

    def extract_topics(self, inbag, outbag, topics):
        self.get_logger().info(f'Processing input bagfile: {inbag}')
        self.get_logger().info(f'Writing to output bagfile: {outbag}')
        self.get_logger().info(f'Extracting topics: {topics}')

        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=inbag, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')  # Empty means no conversion
        reader.open(storage_options, converter_options)

        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)

        topic_types = [(topic, reader.get_metadata().topics_with_message_count[topic].topic_metadata.type) for topic in topics]

        for topic, type in topic_types:
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type, serialization_format='cdr'))

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic in topics:
                writer.write(topic, data, t)

        self.get_logger().info('Closing output bagfile and exit...')

def main(args=None):
    rclpy.init(args=args)
    node = ExtractTopics()

    parser = argparse.ArgumentParser(description='Extracts topics from a ROS2 bagfile into another bagfile.')
    parser.add_argument('inbag', help='input bagfile')
    parser.add_argument('outbag', help='output bagfile')
    parser.add_argument('topics', nargs='+', help='topics to extract')
    parsed_args = parser.parse_args()

    try:
        node.extract_topics(parsed_args.inbag, parsed_args.outbag, parsed_args.topics)
    except Exception as e:
        node.get_logger().error('Failed to extract topics: ' + str(e))
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
