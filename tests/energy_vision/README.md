# energy_vision（蓝胖鱼算法 ROS 2 包）

对接 [robomini_sim](https://github.com/tianze-dev/robomini_sim)（Gazebo Harmonic + ROS 2 Jazzy）。
本包实现中期考核第 1–3 部分，第四部分只需在第 3 部分基础上加开火与提前量。

## 节点与话题

| 节点 | 作用 | 输入 | 输出 |
| --- | --- | --- | --- |
| `camera_node` | 取流/实时显示（帧率/截图/录制，第二部分） | `/camera/image_raw` | — |
| `light_detector` | **方案 A**：HSV 灯条 + 标注 + 位姿（第一、三部分） | `/camera/image_raw` | `/vision/pending_target_pose`、`/vision/rune_states` |
| `yolo_target_node` | **方案 B**：YOLO 检测靶板 + 位姿（第一、三部分） | `/camera/image_raw` | `/vision/pending_target_pose`、`/vision/rune_states` |
| `pose_aim_node` | 位姿 → 瞄准角 + 开火（第三、四部分） | `/vision/pending_target_pose`、`/gimbal_to_vision` | `/vision_to_gimbal` |
| `aim_node` / `aim_follower` / `joint_state_adapter` | 旧版像素伺服链路（**legacy，勿用**） | — | — |

**检测方案二选一**（不要同时开）：

| 方案 | 启动 | 适用 |
| --- | --- | --- |
| A. HSV 灯条 | `ros2 run energy_vision light_detector` | 灯条分离清晰时 |
| B. YOLO | `ros2 run energy_vision yolo_target_node --ros-args -p model:=<best.pt>` | 遮挡/灯条合并、追求稳定 |

两种方案输出**同一个话题格式**，`pose_aim_node` 不用改。

`pose_aim_node` 直接消费 `/vision/pending_target_pose`
（`geometry_msgs/PoseStamped`，`position.x/y/z` 即靶心在相机光轴系的 右/下/前 坐标，米），
内部完成 相机系→云台系 变换、枪口偏移修正、重力补偿、旋转靶提前量，再按仿真接口发 `/vision_to_gimbal`。

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
| `cam_fwd_m` / `cam_up_m` | 0.036 / 0.075 | 相机光心相对俯仰轴的 前向/向上 偏移（m） |
| `muzzle_fwd_m` | 0.095 | 枪口相对俯仰轴的 前向 偏移（m） |
| `fire_mode` | 1 | 0=只跟不射；1=识别到就射；2=对准才射 |
| `fire_align_deg` | 1.5 | 锁定开火：云台追平“带提前量的指令角”的阈值（度） |
| `gravity` | 9.81 | 重力补偿（m/s²），球下坠抬高仰角；0 关闭 |
| `pitch_offset_deg` | 0.0 | 实测标定的固定仰角偏置（度） |
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

## YOLO 方案（可选，替换灯条检测）

当 HSV + 几何配对在遮挡/合并灯条下不稳时，可用一个小 YOLO 直接检测“待激活/已激活”靶板。
**下游 `pose_aim_node` 完全不用改**——YOLO 节点输出同样的 `/vision/pending_target_pose`。

```bash
# 0) 只装一次
pip install ultralytics

# 1) 采帧（仿真跑着、多角度多阶段各采一轮；每类建议 150~300 张）
ros2 run energy_vision collect_frames --ros-args -p out_dir:=yolo_frames -p every:=10

# 2) 用 labelImg 标框，两类：pending(0) / active(1)，导出 YOLO 格式到 yolo/
#    yolo/images/train/*.jpg  yolo/labels/train/*.txt  （val 同理，留 20%）

# 3) 训练（scripts/data.yaml 是配置模板，复制到 yolo/data.yaml 后改 path）
python3 scripts/train_yolo.py --data yolo/data.yaml --epochs 80
#   产物：yolo/runs/rune/weights/best.pt

# 4) 推理运行（与 light_detector 二选一，别同时开）
ros2 run energy_vision yolo_target_node --ros-args \
  -p model:=yolo/runs/rune/weights/best.pt -p conf:=0.35
```

原理：框中心 → 角度，框宽 → 距离（`d = fx·靶板宽/框宽`），再交给 `pose_aim_node` 做瞄准与重力补偿。
调试画面：`/vision/yolo_debug`。

## 第四部分（击打各阶段）

开火不需要额外代码：`pose_aim_node` 默认 `fire_mode=2`（锁定后开火），对准且追平指令角时自动发
`mode=2`，仿真 `launcher_node` 按 3 发/s 节流发射并自己判定命中。球速固定 12 m/s（`launcher_node`
的 `default_speed`），且**已内置重力补偿**（`gravity` 参数，默认 9.81）：球飞行中下坠，仰角自动抬高。

- 看命中：`ros2 topic echo /rune/hit`（`valid=true` 且 `score>0` 才算有效命中）；
- 看总分：`ros2 topic echo /score`（`total`、`mode_hits[0..3]`、`penalty`、`balls_fired`）；
- 分值（人工/自主）：fixed 15/30、random_fixed 20/40、small 25/50、big 30/60；
- **注意罚分**：`/score` 的 `penalty` 是“超发”扣分（每超 1 球 +20），所以务必用 `fire_mode=2`
  锁定开火，别边扫边射浪费弹药。

打各阶段的建议顺序：`fixed`（调稳对准）→ `small`（慢转，调 `dead_reckon_s`/`smooth_alpha`）
→ `random_fixed` → `big`（快转，调 `latency_s` 提前量）。

`/rune/state` 里 `stage`（0=FIXED 1=RANDOM_FIXED 2=SMALL 3=BIG）与 `speed`（机关角速度）
给旋转靶提前量提供了真值；但正式考核中第 1、3 部分必须从图像自己解，`/rune/state` 只用于调试对照，
不要在提交逻辑里直接读它当作识别结果。
