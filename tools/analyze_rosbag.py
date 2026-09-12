#!/usr/bin/env python3
"""Summarize and export ROV-MRT NMPC experiment topics from a ROS 2 bag."""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from experiment_metrics import compute_metrics, format_summary, write_summary


TOPICS = {
    '/nmpc/diagnostics': 'diagnostics.csv',
    '/rov_mrt/state': 'state.csv',
    '/rov_mrt/control': 'control.csv',
}


def read_selected_topics(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    type_map = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    missing = sorted(set(TOPICS) - set(type_map))
    if missing:
        raise RuntimeError(
            'Missing required topics: ' + ', '.join(missing)
        )

    message_types = {
        topic: get_message(type_map[topic])
        for topic in TOPICS
    }
    records = {topic: [] for topic in TOPICS}

    while reader.has_next():
        topic, serialized_data, timestamp = reader.read_next()
        if topic not in TOPICS:
            continue
        message = deserialize_message(
            serialized_data,
            message_types[topic],
        )
        records[topic].append(
            (timestamp * 1.0e-9, list(message.data))
        )

    return records


def make_array(records, topic, expected_width):
    topic_records = records[topic]
    if not topic_records:
        raise RuntimeError(f'No messages recorded for {topic}')

    first_time = topic_records[0][0]
    rows = []
    for timestamp, values in topic_records:
        if len(values) != expected_width:
            raise RuntimeError(
                f'{topic} expected {expected_width} values, '
                f'got {len(values)}'
            )
        rows.append([timestamp - first_time] + values)
    return np.asarray(rows, dtype=float)


def save_csv(path, data, header):
    np.savetxt(
        path,
        data,
        delimiter=',',
        header=header,
        comments='',
        fmt='%.10g',
    )


def summarize(bag_path, output_directory, deadline):
    records = read_selected_topics(bag_path)

    diagnostics = make_array(
        records, '/nmpc/diagnostics', 6
    )
    states = make_array(records, '/rov_mrt/state', 5)
    controls = make_array(records, '/rov_mrt/control', 2)

    output_directory.mkdir(parents=True, exist_ok=True)

    save_csv(
        output_directory / TOPICS['/nmpc/diagnostics'],
        diagnostics,
        'time_s,gB_physical_m2,gR_physical_m2,'
        'max_safety_slack_m2,solve_time_s,'
        'max_constraint_violation,avoidance_active',
    )
    save_csv(
        output_directory / TOPICS['/rov_mrt/state'],
        states,
        'time_s,X_M_m,Y_M_m,psi_M_rad,v_M_mps,delta_R_rad',
    )
    save_csv(
        output_directory / TOPICS['/rov_mrt/control'],
        controls,
        'time_s,a_M_mps2,omega_delta_radps',
    )

    metrics = compute_metrics(
        diagnostics, states, controls, deadline=deadline
    )
    write_summary(output_directory / 'summary.json', metrics)
    print('\n' + format_summary(metrics, bag_path))
    print(f'\nCSV/JSON output directory:\n{output_directory}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'bag_path',
        type=Path,
        help='ROS 2 bag directory containing metadata.yaml',
    )
    parser.add_argument(
        '--deadline',
        type=float,
        default=0.30,
        help='Controller deadline used for overrun counting.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='CSV output directory (default: BAG_PATH/analysis)',
    )
    arguments = parser.parse_args()

    bag_path = arguments.bag_path.expanduser().resolve()
    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(
            f'No metadata.yaml found in {bag_path}'
        )
    output_directory = (
        arguments.output.expanduser().resolve()
        if arguments.output is not None
        else bag_path / 'analysis'
    )
    summarize(bag_path, output_directory, arguments.deadline)


if __name__ == '__main__':
    main()
