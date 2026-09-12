# Changelog

## 0.1.2 - 2026-09-12

- Keep NumPy below 2.0 for compatibility with the validated ROS 2/SciPy environment.
- Reuse compatible system Matplotlib versions instead of installing a duplicate copy.
- Force the non-interactive Agg backend for deterministic headless result plotting.

## 0.1.1 - 2026-09-12

- Prevent publishing after ROS context shutdown when SIGINT interrupts IPOPT.
- Avoid duplicate `rclpy.shutdown()` calls during launch termination.
- Make the repository check script use `python3` when no virtual environment is active.

## 0.1.0 - 2026-09-12

- Initial reproducible ROS 2 NMPC demo release.
