# Reproduction protocol

## Validated environment

The reference run used Ubuntu 24.04, ROS 2 Jazzy, CasADi/IPOPT, and RViz2 in a
VMware virtual machine. The controller period was 0.30 s.

## Data layouts

- State: `[X_M_m, Y_M_m, psi_M_rad, v_M_mps, delta_R_rad]`
- Control: `[a_M_mps2, omega_delta_radps]`
- Each obstacle: `[x_m, y_m, vx_mps, vy_mps, radius_m]`
- Diagnostics: `[MRT_margin_m2, ROV_margin_m2, max_slack_m2,
  solve_time_s, max_constraint_violation, avoidance_active]`

## Acceptance checks

For the unchanged default configuration, a successful repeated run should:

- complete activation, side transition, release, and road recovery;
- retain positive sampled physical margins for both bodies;
- report maximum constraint violation no larger than `1e-3`;
- stay within configured actuator and state limits;
- have no optimizer call exceed the 0.30 s period on the tested platform.

Exact trajectories and solve times can vary with CPU load, CasADi/IPOPT
version, and virtual-machine scheduling. Do not require bitwise equality with
the included CSV files.

## Verification commands

```bash
python -m compileall -q src/rov_mrt_sim tools
PYTHONPATH=src/rov_mrt_sim pytest -q src/rov_mrt_sim/test
python tools/evaluate_results.py results/opposite_side
python tools/plot_results.py results/opposite_side
```

For a new bag, use `tools/analyze_rosbag.py`. It requires a sourced ROS 2 Jazzy
environment because it imports rosbag message-support modules.
