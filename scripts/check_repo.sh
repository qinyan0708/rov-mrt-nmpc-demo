#!/usr/bin/env bash
set -euo pipefail

python3 -m compileall -q src/rov_mrt_sim tools
PYTHONPATH=src/rov_mrt_sim python3 -m unittest discover -s src/rov_mrt_sim/test -v
python3 tools/evaluate_results.py results/opposite_side
python3 tools/plot_results.py results/opposite_side
