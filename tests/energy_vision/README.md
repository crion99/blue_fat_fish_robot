# energy_vision（蓝胖鱼算法 ROS 2 包）

对接 [robomini_sim](https://github.com/tianze-dev/robomini_sim)（Gazebo Harmonic + ROS 2 Jazzy）。
本包实现中期考核第 1–3 部分，第四部分只需在第 3 部分基础上加开火与提前量。

## 节点与话题

| 节点 | 作用 | 输入 | 输出 |
| --- | --- | --- | --- |
| `camera_node` | 取流/实时显示（帧率/截图/录制，第二部分） | `/camera/image_raw` | — |
| `light_detector` | 灯效识别 + 标注 + 位姿（第一、三部分） | `/camera/image_raw` | `/vision/pending_target_pose`、`/vision/rune_states` |
| `pose_aim_node` | 位姿 + 瞄准（第三部分） | `/vision/pending_target_pose`、`/gimbal_to_vision` | `/vision_to_gimbal` |
| `aim_node` | 旧版像素伺服（参考） | `/vision/pending_target_pose` | `/aim/joint_target` |

`pose_aim_node` 直接消费 `light_detector` 发布的 `/vision/pending_target_pose`
（`geometry_msgs/PoseStamped`，`position.x/y/z` 即靶心在相机光轴系的 右/下/前 坐标，米），
内部完成 相机系→云台系 变换、枪口偏移修正、旋转靶提前量，再按仿真接口发 `/vision_to_gimbal`。
无需角点话题。

## 构建（在 Ubuntu 24.04 上）

```bash
# 1) 把本包软链进仿真工作区（不改仿真仓库本身）
ln -s "$(pwd)/tests/energy_vision" algorithm/robomini_sim/ros2_ws/src/energy_vision

# 2) 构建仿真（含接口）+ 本包
cd algorithm/robomini_sim && ./scripts/build.sh

# 3) 只构建本包
cd algorithm/robomini_sim/ros2_ws
source install/setup.bash
colcon build --packages-select energy_vision
```

## 运行

```bash
source /opt/ros/jazzy/setup.bash
source algorithm/robomini_sim/ros2_ws/install/setup.bash

ros2 launch robomini_sim sim.launch.py side:=blue control_mode:=auto gui:=true
ros2 run energy_vision camera_node
ros2 run energy_vision light_detector
ros2 run energy_vision pose_aim_node
```

`pose_aim_node` 参数（`ros2 run energy_vision pose_aim_node --ros-args -p ...`）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `t_cam_x` / `t_muzzle_x` | 0.00 / 0.10 | 相机、枪口沿枪管方向的安装偏移（m） |
| `fire_mode` | 1 | 1=只跟瞄；2=跟瞄并开火 |
| `latency_s` | 0.03 | 图像→出弹总延迟，用于旋转靶提前量 |
| `stale_timeout_s` | 0.40 | 目标丢失超过该时长后发 `mode=0` 停控 |
| `dead_reckon_s` | 0.50 | 丢失后按角速度航位推算、保持扫动的时长 |
| `smooth_alpha` | 0.4 | 指令低通系数（0~1），越小越“钝”、越不易过冲 |
| `max_cmd_rate_radps` | 3.0 | 指令限速（rad/s），过大容易把目标甩出视野 |

## 第一、二部分验证

- 第二部分：`camera_node` 窗口应稳定显示画面并叠加帧率；按 `s` 存图、`r` 录一段演示视频即可作为“实时显示”证据。
- 第一部分：`light_detector` 画面里每个靶的框与 `Pending/Active` 文字就是识别标注结果；
  识别出的灯态数量会发到 `/vision/rune_states`（形如 `pending=1 active=4`），
  可和裁判真值 `/rune/state` 的 `targets` 字段逐帧比对：

  ```bash
  ros2 topic echo /vision/rune_states
  ros2 topic echo /rune/state --field targets   # 真值，仅调试对照，不要进提交逻辑
  ```

## 与第四部分的衔接

`/rune/state` 里 `stage`（0=FIXED 1=RANDOM_FIXED 2=SMALL 3=BIG）与 `speed`（机关角速度）
给旋转靶提前量提供了真值；但正式考核中第 1、3 部分必须从图像自己解，`/rune/state` 只用于调试对照，
不要在提交逻辑里直接读它当作识别结果。
