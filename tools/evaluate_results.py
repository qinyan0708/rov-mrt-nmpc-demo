#!/usr/bin/env python3
"""Recompute metrics from exported CSV files without requiring ROS 2."""

import argparse
from pathlib import Path

from experiment_metrics import (
    compute_metrics,
    format_summary,
    load_csv_directory,
    write_summary,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('analysis_directory', type=Path)
    parser.add_argument('--deadline', type=float, default=0.30)
    arguments = parser.parse_args()
    arrays = load_csv_directory(arguments.analysis_directory)
    metrics = compute_metrics(*arrays, deadline=arguments.deadline)
    write_summary(arguments.analysis_directory / 'summary.json', metrics)
    print(format_summary(metrics, arguments.analysis_directory))


if __name__ == '__main__':
    main()
