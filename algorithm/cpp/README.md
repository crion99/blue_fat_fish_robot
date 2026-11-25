# 算法 C++ 工程（中期考核第 1–3 部分）

Ubuntu 20.04+ / 22.04，C++17，依赖 OpenCV 4（`core imgproc highgui videoio imgcodecs`）。

```bash
sudo apt install -y build-essential cmake libopencv-dev
cmake -S algorithm/cpp -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure     # 1、3 两部分的算法冒烟测试
```

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/bff/linalg.hpp` | 3×3/6×6/9×9 线性代数、SO(3) 指数/对数（无第三方依赖） |
| `include/bff/{camera,pnp,aim,pipeline}.hpp` | 第三部分：相机模型、PnP、瞄准解算、流程封装 |
| `include/bff/{image,color,blobs,rune_detector}.hpp` | 第一部分：灯效识别 |
| `include/bff/{camera_source,config}.hpp` | 第二部分：OpenCV 采集与 YAML 配置 |
| `apps/live_view.cpp` | 第二部分的实时显示，叠加第一/第三部分结果 |
| `apps/{pose,rune}_selftest.cpp` | 合成数据验证，输出 `build/out/report.md` |
| `configs/*.yml` | 内参、外参、靶标尺寸、灯效阈值 |

## 三部分怎么跑、怎么验

**第 2 部分（相机 + 实时显示）**

```bash
./build/live_view --source 0 --width 1280 --height 720 --fps 60
./build/live_view --source 0 --record demo.avi        # 录一段演示
```

看 HUD 帧率是否稳定；按 `s` 截图、`r` 录/停、`q` 退出。

**第 1 部分（灯效识别）**

```bash
./build/live_view --source rune_inactive.mp4 --csv inact.csv
./build/live_view --source rune_active.mp4   --csv act.csv
```

画面上的方框与 `ACTIVE/INACTIVE xx%` 就是标注结果；CSV 每帧一行，用于统计准确率、
切换延迟与空场景误检。填 `tests/2026-11-25_算法_第一部分_灯效识别实测记录模板.md`。

**第 3 部分（位姿与瞄准）**

```bash
./build/live_view --source 0 --pose --yaw 0 --pitch 0
```

按 `p` 进入点选模式，依次点靶面 4 个角（左上→右上→右下→左下），画面显示位姿 RMSE、
距离、`Δyaw/Δpitch` 以及一行 `AIM,...` 串口报文。误差验证按
`algorithm/docs/03-靶位姿解算与瞄准转角.md` 的 3.7.2 做（卷尺 + 量角器 + 激光笔），
记录填 `tests/2026-11-25_算法_第三部分_位姿与瞄准实测记录模板.md`。

## 参数标定顺序（很重要）

1. 相机内参 → `configs/camera.yml`（棋盘格标定，重投影 < 0.3 px）；
2. 靶标实际边长 → `configs/target.yml`（以规则手册为准，量一次）；
3. 相机-云台外参 → `configs/gimbal.yml`：先按图纸填，再用激光落点法反解修正；
4. 灯效阈值 → `configs/rune.yml`：先用实拍片段看 CSV 里的 `meanV/blinkHz/modulation`，
   再定 `vMin/sMin/hue*/modulationOn`。

改完配置不用重新编译。

## 说明

- 本机（Windows，无 OpenCV）只验证了第 1、3 部分的算法核心与两个自检程序；
  `live_view` 依赖 OpenCV，需在 Ubuntu 上首次编译时确认 API 细节。
- 第三部分目前只解算静态靶；第四部分打旋转靶时按 `doc 3.9` 加提前量。
