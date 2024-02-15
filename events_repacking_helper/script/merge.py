#!/usr/bin/env python3

import rosbag2_py
import argparse
import os
from rosbag2_py import Reader, Writer, StorageOptions, ConverterOptions, TopicMetadata

def merge(inbags, outbag='output.bag', topics=None, exclude_topics=[]):
    writer_options = StorageOptions(uri=outbag, storage_id='sqlite3')
    writer = Writer()
    writer.open(writer_options, ConverterOptions('', ''))

    for inbag in inbags:
        print(f'Processing input bagfile: {inbag}')
        reader_options = StorageOptions(uri=inbag, storage_id='sqlite3')
        reader = Reader()
        reader.open(reader_options, ConverterOptions('', ''))

        if topics is None:
            topics = [topic.topic_metadata.name for topic in reader.get_all_topics_and_types()]

        topics_to_write = set(topics) - set(exclude_topics)

        for topic in reader.get_all_topics_and_types():
            if topic.topic_metadata.name in topics_to_write:
                writer.create_topic(TopicMetadata(
                    name=topic.topic_metadata.name,
                    type=topic.topic_metadata.type,
                    serialization_format=topic.topic_metadata.serialization_format))

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic in topics_to_write:
                writer.write(topic, data, t)

    print(f'Saving output bag file: {outbag}')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Merge multiple ROS2 bag files into a single one.')
    parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
    parser.add_argument('--output', help='output bag file', default='output.bag')
    parser.add_argument('--topics', help='topics to merge from the input bag files', nargs='+', default=None)
    parser.add_argument('--exclude_topics', help='topics not to merge from the input bag files', nargs='+', default=[])
    args = parser.parse_args()

    merge(args.inbag, args.output, args.topics, args.exclude_topics)
