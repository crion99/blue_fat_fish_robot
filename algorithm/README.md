# 算法 / 视觉方向

对接 [robomini_sim](https://github.com/tianze-dev/robomini_sim)（Gazebo Harmonic + ROS 2 Jazzy）。
代码分两块：**ROS 2 算法包**（对接仿真）与**纯 C++ 核心**（不依赖 ROS 的数学自检）。

## 目录

```text
algorithm/
├── docs/
│   └── 03-靶位姿解算与瞄准转角.md     # 第三部分技术文档（含实测误差数据）
├── cpp/                              # 纯 C++17 核心：无 ROS 依赖，可跑 ctest 自检
│   ├── include/bff/                  # 相机模型 / PnP / 瞄准 / 灯效识别 / 图像处理
│   ├── src/                          # 对应实现
│   ├── apps/live_view.cpp            # 单机实时显示（叠加识别与瞄准结果）
│   ├── apps/pose_selftest.cpp        # 第三部分合成蒙特卡洛
│   └── apps/rune_selftest.cpp        # 第一部分合成评估
└── robomini_sim/                     # 第三方仿真平台（单独 clone，不入库）
```

ROS 2 算法包在 [`tests/energy_vision/`](../tests/energy_vision/README.md)。

## 考核任务与实现位置

| # | 任务 | 实现 | 验证方式 |
| --- | --- | --- | --- |
| 1 | 识别待激活 / 已激活灯效并标注（10 分） | `light_detector`（HSV，按簇灯条数判：4 根=pending、2 根=active）；或 `yolo_target_node`（YOLO） | 画面标注 + `/vision/rune_states`，对 `/rune/state` 真值比对 |
| 2 | 调用相机并实时显示（10 分） | `camera_node`（帧率、截图、录制） | 窗口画面 + 录像 |
| 3 | 靶位姿与瞄准转角（10 分） | `light_detector`/`yolo_target_node` 出位姿 → `pose_aim_node` 算 yaw/pitch | `/vision_to_gimbal` + 云台实际跟随；`cpp` 侧有蒙特卡洛误差表 |
| 4 | 仿真各阶段击打（40 分） | `pose_aim_node` 的 `fire_mode`（1=识别即射、2=对准才射） | `/rune/hit`（valid=true）、`/score` |

## 快速开始（Ubuntu 24.04）

```bash
# 1) 编译
sudo apt install -y build-essential cmake libopencv-dev
cmake -S algorithm/cpp -B build && cmake --build build -j   # 纯 C++ 核心 + 自检
ctest --test-dir build --output-on-failure

# 2) ROS 2 包（需先装 ROS 2 Jazzy + Gazebo Harmonic）
ln -s "$(pwd)/tests/energy_vision" algorithm/robomini_sim/ros2_ws/src/energy_vision
cd algorithm/robomini_sim && ./scripts/build.sh

# 3) 跑仿真 + 算法
ros2 launch robomini_sim sim.launch.py side:=blue control_mode:=auto gui:=true
ros2 run energy_vision light_detector        # 或 yolo_target_node
ros2 run energy_vision pose_aim_node
```

完整操作手册见 [`tests/energy_vision/README.md`](../tests/energy_vision/README.md)，
端到端测试步骤见 [`tests/energy_vision/TESTING.md`](../tests/energy_vision/TESTING.md)。

## 设计要点

- **相机与枪管同向**，外参只剩固定平移；`pose_aim_node` 按真实安装几何解算（相机在俯仰轴上方 7.5cm），
  并做重力下坠补偿（`gravity`）与旋转靶提前量（`latency_s`）。
- **检测有两条路**：HSV 灯条法（无依赖、易调）与 YOLO（鲁棒、需训练）。
  两者输出同一话题格式，下游完全复用。
- **纯 C++ 核心**可脱离 ROS 单独验证数学与误差，`ctest` 覆盖 PnP、瞄准闭环与灯效识别。
