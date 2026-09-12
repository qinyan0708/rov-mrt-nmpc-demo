# ROV–MRT 采矿车动态避障 NMPC Demo

本仓库是一个面向 ROV–MRT 铰接式深海采矿车的 ROS 2 Jazzy 可复现仿真项目，
实现 S 形道路跟踪、双动态障碍预测、异侧连续绕行、全车体圆包络安全约束、
RViz 演示、rosbag 记录与结果复算。

当前定位是研究型工程 Demo，而不是通用动态避障规划器或实车安全系统。
默认绕行侧和换侧区间根据该工况预先设计，预测模型与仿真对象均采用同一套
五维运动学模型。

## 快速运行

环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rov_mrt_sim rov_mrt_demo.launch.py
```

无 RViz 运行：

```bash
ros2 launch rov_mrt_sim rov_mrt_demo.launch.py use_rviz:=false
```

## 复算已记录结果

```bash
python tools/evaluate_results.py results/opposite_side
python tools/plot_results.py results/opposite_side
```

参考实验包含 226 次 NMPC 求解：MRT 和 ROV 最小物理裕度分别为
`2.468911 m²` 与 `10.298939 m²`，最大约束违反量为 0，平均求解时间
`0.072215 s`，最大求解时间 `0.292165 s`，没有超过 `0.30 s` 控制周期。

上述裕度是圆包络模型在离散记录时刻的平方距离裕度；求解时间只统计优化器
调用，不代表端到端硬实时性能。

## 阅读顺序

1. `docs/mathematical_model.md`：状态、输入、RK4 和碰撞约束；
2. `docs/controller_design.md`：NMPC 目标函数、换侧走廊与失败回退；
3. `src/rov_mrt_sim/config/opposite_side.yaml`：默认参数；
4. `nmpc_avoidance_controller_node.py`：求解器构建与闭环回调；
5. `docs/reproduction.md`：数据格式、录制和成功判据；
6. `docs/limitations.md`：论文级扩展方向。

公开前请先阅读英文主 README 中的完整适用边界与许可证说明。
