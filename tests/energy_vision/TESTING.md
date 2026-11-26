# 完整测试流程（中期考核 算法第 1–4 部分）

> 假设：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 已装好，仿真在 `~/projects/robomini_sim`。
> 下面路径按此写死，实际路径不同就替换。

---

## 0. 编译（首次 + 每次改代码后）

```bash
# 首次：把算法包软链进仿真工作区（只做一次）
ln -s "$(pwd)/tests/energy_vision" ~/projects/robomini_sim/ros2_ws/src/energy_vision

# 每次改代码后重编
cd ~/projects/robomini_sim
./scripts/build.sh                                   # 全量（首次）
# 或只编算法包（快）：
cd ~/projects/robomini_sim/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select energy_vision
```

改完 `.cpp` 后**必须重编 + 重新 `source install/setup.bash` + 重启节点**才生效。

---

## 1. 启动仿真（终端 A）

```bash
cd ~/projects/robomini_sim
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# 有独显：render_backend:=gpu；无独显/WSL：先 export 再启动
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch robomini_sim sim.launch.py side:=blue control_mode:=auto gui:=true
# 服务器无头：把 gui:=true 换成 gui:=false
```

看到 Gazebo 场地和能量机关出来、没有红字报错，就是起来了。

---

## 2. 启动算法节点（终端 B / C / D，各开一个）

每个终端都要先 source：

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/robomini_sim/ros2_ws/install/setup.bash
```

然后分别运行：

```bash
# 终端 B —— 第2部分：相机实时显示（q 退出 / s 截图 / r 录制）
ros2 run energy_vision camera_node

# 终端 C —— 第1部分（灯效识别+标注）+ 第3部分（位姿解算）
ros2 run energy_vision light_detector

# 终端 D —— 第3部分（瞄准角）+ 第4部分（锁定开火）
ros2 run energy_vision pose_aim_node
```

`pose_aim_node` 默认 `fire_mode=2`（对准且追平指令才开火），跑起来就自动跟瞄 + 自动开火。

---

## 3. 验证第 2 部分（相机 + 实时显示）

- `camera_node` 窗口应有画面，左上角有 `xx.x fps 1280x720`；
- 按 `s` 存 `snapshot.png`，按 `r` 录 `camera.avi`（这就是提交视频的素材）；
- 看帧率是否够：

```bash
ros2 topic hz /camera/image_raw   # 跑 10 秒看平均，理想 55~60
```

帧率低见 README 的「帧率低」处理（降分辨率 / GPU 渲染 / 关 GUI）。

---

## 4. 验证第 1 部分（灯效识别 + 标注）

先把靶子停到相机能看见的地方：

```bash
ros2 service call /rune/set_mode robomini_interfaces/srv/SetRuneMode "{mode: fixed}"
```

- `light_detector` 窗口里应出现灯条、靶框，框上标 `Pending` / `Active`；
- 识别结果在话题上，和真值逐帧对照：

```bash
ros2 topic echo /vision/rune_states          # 我们识别的：pending=1 active=4
ros2 topic echo /rune/state --field targets  # 裁判真值（只调试用，别进提交逻辑）
```

两者一致 = 第一部分过。

---

## 5. 验证第 3 部分（位姿 + 瞄准）

- `light_detector` 窗口出现 `PENDING | Pose valid`、`Reprojection RMSE < 1 px`；
- `pose_aim_node` 终端不再刷「目标丢失」；
- 看下发的指令：

```bash
ros2 topic echo /vision_to_gimbal   # 应有 mode=1/2 和 yaw/pitch
```

- 看 Gazebo 里枪管是否平滑转过去对准靶心。若方向反了，改 `pose_aim_node.cpp` 里 `gy=-cx` / `gz=-cy` 的正负号。

---

## 6. 验证第 4 部分（各阶段击打）

`pose_aim_node` 锁定后自动发 `mode=2`，仿真按 3 发/s 开火。看命中与计分：

```bash
ros2 topic echo /rune/hit    # valid=true 且 score>0 才算有效命中
ros2 topic echo /score       # total / mode_hits[0..3] / penalty / balls_fired
```

分值（人工/自主）：fixed 15/30、random_fixed 20/40、small 25/50、big 30/60。
`penalty` 是超发罚分，务必用 `fire_mode=2` 锁定开火。

---

## 7. 各阶段切换命令

```bash
ros2 service call /rune/set_mode robomini_interfaces/srv/SetRuneMode "{mode: fixed}"
ros2 service call /rune/set_mode robomini_interfaces/srv/SetRuneMode "{mode: random_fixed}"
ros2 service call /rune/set_mode robomini_interfaces/srv/SetRuneMode "{mode: small, a: 0.0, omega: 0.1}"   # 慢转，先调稳
ros2 service call /rune/set_mode robomini_interfaces/srv/SetRuneMode "{mode: big, a: 0.0, omega: 0.0}"     # 快转，a/omega=0 表示随机合法值

ros2 service call /rune/next_stage std_srvs/srv/Trigger   # 手动推进一阶段
ros2 service call /rune/reset      std_srvs/srv/Trigger   # 复位机关
ros2 service call /sim/reset       std_srvs/srv/Trigger   # 整体复位
```

建议顺序：`fixed`（调对准+开火闭环）→ `small`（慢转，调 `dead_reckon_s/smooth_alpha`）→ `random_fixed` → `big`（快转，调 `latency_s` 提前量）。

---

## 8. 录证据

```bash
ros2 bag record -o midterm_evidence \
  /camera/image_raw /vision/rune_states /vision_to_gimbal /rune/hit /score
# Ctrl+C 结束
```

再配合 OBS/屏幕录制，把「视觉窗口 + Gazebo 枪管跟瞄命中」录下来；`camera_node` 按 `r` 也能录。

---

## 9. `pose_aim_node` 参数速查

| 参数 | 默认 | 用途 |
| --- | --- | --- |
| `fire_mode` | 2 | 0=只跟；1=跟踪即射；2=锁定后射 |
| `fire_align_deg` | 1.5 | 开火：云台追平指令角的阈值（度） |
| `t_cam_x` / `t_muzzle_x` | 0.00 / 0.10 | 相机、枪口沿枪管安装偏移（m） |
| `latency_s` | 0.03 | 提前量：图像→出弹延迟 |
| `smooth_alpha` | 0.4 | 指令低通，越小越稳 |
| `max_cmd_rate_radps` | 3.0 | 指令限速 |
| `stale_timeout_s` / `dead_reckon_s` | 0.40 / 0.50 | 丢帧判停 / 航位推算时长 |

运行时覆盖：`ros2 run energy_vision pose_aim_node --ros-args -p latency_s:=0.05`

---

## 10. 常见问题速查

| 现象 | 处理 |
| --- | --- |
| 目标丢失刷屏 | 已修（时间域统一）；确认 light_detector 有 `Pending` |
| 云台猛摆冲出视野 | 降 `max_cmd_rate_radps`、`smooth_alpha` |
| 追得上打不中（差固定一点） | 调 `latency_s`（偏前减小、偏后增大） |
| 旋转靶不开火 | 已修（改判“追平指令角”）；`fire_mode:=2` |
| 帧率低 | 降相机分辨率 / GPU 渲染 / 关 GUI |
| 云台方向反 | `pose_aim_node.cpp` 里 `gy/gz` 取反 |
