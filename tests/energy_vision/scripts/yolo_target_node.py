#!/usr/bin/env python3
"""YOLO 检测待激活靶板 -> 位姿 -> 瞄准/开火，并把「四个部分的显示」做在同一张调试图上。

输出：
    /vision/pending_target_pose  PoseStamped  靶心在相机系的位置（m）
    /vision/rune_states          String       "pending=N active=M"
    /vision/yolo_debug           Image        四部分合一的可视化（rqt_image_view 看）
                                             [P2] 帧率/分辨率
                                             [P1] 待激活/已激活 数量 + 检测框
                                             [P3] 距离 + 相对角 + 云台指令
                                             [P4] mode（1=只跟 2=开火）

用法：
    ros2 run energy_vision yolo_target_node            # 用包内模型
    ros2 run energy_vision yolo_target_node --ros-args -p model:=/path/to/best.pt
"""
import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO

try:
    from robomini_interfaces.msg import VisionToGimbal
except ImportError:  # 接口包未构建时仍可单独调试检测
    VisionToGimbal = None

PENDING = 0
ACTIVE = 1


def _default_model_path():
    """包内自带模型：share/energy_vision/models/rune_best.pt"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('energy_vision'), 'models', 'rune_best.pt')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return ''


class YoloTargetNode(Node):
    def __init__(self):
        super().__init__('yolo_target_node')
        self.declare_parameter('model', '')
        self.declare_parameter('conf', 0.30)
        self.declare_parameter('plate_width_m', 0.195)
        self.declare_parameter('nominal_distance_m', 1.9)  # 0 = 用框宽估算
        self.declare_parameter('center_alpha', 0.5)
        self.declare_parameter('target_gate_px', 120.0)  # 目标连续性门限：超出行走距离才换靶
        self.declare_parameter('publish_debug_image', True)

        model_path = self.get_parameter('model').value or _default_model_path()
        if not model_path or not os.path.exists(model_path):
            raise RuntimeError('找不到 YOLO 模型，请用 -p model:=/path/to/best.pt 指定')
        self.model = YOLO(model_path)
        self.conf = float(self.get_parameter('conf').value)
        self.plate_w = float(self.get_parameter('plate_width_m').value)
        self.nominal_d = float(self.get_parameter('nominal_distance_m').value)
        self.center_alpha = float(self.get_parameter('center_alpha').value)
        self.target_gate_px = float(self.get_parameter('target_gate_px').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self._locked_uv = None  # 目标连续性：上一次选中的靶心像素

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = 0.0
        self._uv = None
        self._fps_t = time.monotonic()
        self._fps_n = 0
        self._fps = 0.0
        self._last_cmd = None  # (mode, yaw_deg, pitch_deg)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)
        if VisionToGimbal is not None:
            self.create_subscription(VisionToGimbal, '/vision_to_gimbal', self.on_cmd, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/pending_target_pose', 10)
        self.state_pub = self.create_publisher(String, '/vision/rune_states', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/yolo_debug', 10)
        self.get_logger().info(f'yolo_target_node 启动，模型: {model_path}，标称距离 {self.nominal_d} m')

    def on_info(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def on_cmd(self, msg):
        self._last_cmd = (int(msg.mode), math.degrees(msg.yaw), math.degrees(msg.pitch))

    def _draw(self, canvas, pending, active, dist, rel_yaw, rel_pitch, has_target,
              target_uv=None):
        if target_uv is not None:
            # 用绿圈标出“当前锁定的那块靶”，方便肉眼确认没在多块靶之间跳
            cv2.circle(canvas, (int(target_uv[0]), int(target_uv[1])), 18, (0, 255, 0), 3)
        self._fps_n += 1
        now = time.monotonic()
        if now - self._fps_t >= 1.0:
            self._fps = self._fps_n / (now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now

        lines = [
            f'[P2] {self._fps:4.1f} fps  {canvas.shape[1]}x{canvas.shape[0]}',
            f'[P1] pending={pending} active={active}',
        ]
        if has_target:
            lines.append(f'[P3] d={dist:.2f}m rel=({rel_yaw:+.1f}, {rel_pitch:+.1f}) deg')
        else:
            lines.append('[P3] no pending target')
        if self._last_cmd is not None:
            m, yw, pt = self._last_cmd
            tag = 'FIRE' if m == 2 else ('TRACK' if m == 1 else 'IDLE')
            lines.append(f'[P3/P4] cmd yaw={yw:+.1f} pitch={pt:+.1f}  mode={m}({tag})')

        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (14, 32 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

    def on_img(self, msg):
        if self.fx <= 0.0:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result = self.model.predict(img, conf=self.conf, verbose=False)[0]

        pending = 0
        active = 0
        cands = []  # (conf, u, v, w_px)
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            if cls == PENDING:
                pending += 1
                cands.append((conf, 0.5 * (x1 + x2), 0.5 * (y1 + y2), x2 - x1))
            elif cls == ACTIVE:
                active += 1

        # 目标连续性：多块靶同时可见时，优先沿用上一次选中的那块，避免在靶之间来回跳
        best = None
        if cands:
            if self._locked_uv is not None:
                lu, lv = self._locked_uv
                nearest = min(cands, key=lambda c: (c[1] - lu) ** 2 + (c[2] - lv) ** 2)
                dist = math.hypot(nearest[1] - lu, nearest[2] - lv)
                best = nearest if dist <= self.target_gate_px else max(cands, key=lambda c: c[0])
            else:
                best = max(cands, key=lambda c: c[0])
            self._locked_uv = (best[1], best[2])
        else:
            self._locked_uv = None  # 目标消失，下次重新选

        state = String()
        state.data = f'pending={pending} active={active}'
        self.state_pub.publish(state)

        if best is None:
            if self.publish_debug:
                self._draw(result.plot(), pending, active, 0.0, 0.0, 0.0, False)
            return

        _, u, v, w_px = best
        if self._uv is None:
            self._uv = (u, v)
        else:
            a = self.center_alpha
            self._uv = (self._uv[0] + a * (u - self._uv[0]),
                        self._uv[1] + a * (v - self._uv[1]))
        u, v = self._uv

        distance = (self.nominal_d if self.nominal_d > 0.0
                    else self.fx * self.plate_w / max(w_px, 1.0))
        rel_yaw = -math.degrees(math.atan2(u - self.cx, self.fx))
        rel_pitch = -math.degrees(math.atan2(v - self.cy, self.fy))

        if self.publish_debug:
            self._draw(result.plot(), pending, active, distance, rel_yaw, rel_pitch, True,
                       target_uv=(u, v))

        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = (u - self.cx) / self.fx * distance
        out.pose.position.y = (v - self.cy) / self.fy * distance
        out.pose.position.z = distance
        out.pose.orientation.w = 1.0
        self.pose_pub.publish(out)


def main():
    rclpy.init()
    node = YoloTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
