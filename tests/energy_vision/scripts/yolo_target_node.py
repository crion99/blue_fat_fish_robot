#!/usr/bin/env python3
"""用训练好的 YOLO 检测待激活靶板，输出与 light_detector 完全相同的位姿话题，
   下游 pose_aim_node / 瞄准链一行都不用改。

默认加载包内自带的模型（models/rune_best.pt），无需手动指定。

输出：
    /vision/pending_target_pose  geometry_msgs/PoseStamped
        position = 靶心在相机光轴系下的坐标（x 右, y 下, z 前，单位 m）
    /vision/rune_states          std_msgs/String           "pending=N active=M"
    /vision/yolo_debug           sensor_msgs/Image         带框的调试图

原理（单目，瞄准只需要角度 + 距离）：
    - 框中心 (u,v) -> 方向：x = (u-cx)/fx*d, y = (v-cy)/fy*d
    - 框宽 w_px -> 距离：d = fx * plate_width_m / w_px
    pose_aim_node 再做 相机->枪口、重力补偿、旋转靶提前量。

用法：
    ros2 run energy_vision yolo_target_node            # 用包内模型
    ros2 run energy_vision yolo_target_node --ros-args -p model:=/path/to/best.pt
"""
import os

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from ultralytics import YOLO

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
        self.declare_parameter('plate_width_m', 0.195)  # 靶板真实宽度（m）
        # 本仿真里机关位置固定，用固定距离最稳；设为 0 则用框宽估算（会抖）
        self.declare_parameter('nominal_distance_m', 1.9)
        self.declare_parameter('center_alpha', 0.5)     # 框中心低通
        self.declare_parameter('publish_debug_image', True)

        model_path = self.get_parameter('model').value or _default_model_path()
        if not model_path or not os.path.exists(model_path):
            raise RuntimeError('找不到 YOLO 模型，请用 -p model:=/path/to/best.pt 指定')
        self.model = YOLO(model_path)
        self.conf = float(self.get_parameter('conf').value)
        self.plate_w = float(self.get_parameter('plate_width_m').value)
        self.nominal_d = float(self.get_parameter('nominal_distance_m').value)
        self.center_alpha = float(self.get_parameter('center_alpha').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self._uv = None  # 框中心低通状态

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = 0.0

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/pending_target_pose', 10)
        self.state_pub = self.create_publisher(String, '/vision/rune_states', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/yolo_debug', 10)
        self.get_logger().info(f'yolo_target_node 启动，模型: {model_path}')

    def on_info(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def on_img(self, msg):
        if self.fx <= 0.0:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result = self.model.predict(img, conf=self.conf, verbose=False)[0]

        pending = 0
        active = 0
        best = None  # (conf, u, v, w_px)
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            if cls == PENDING:
                pending += 1
                if best is None or conf > best[0]:
                    best = (conf, 0.5 * (x1 + x2), 0.5 * (y1 + y2), x2 - x1)
            elif cls == ACTIVE:
                active += 1

        state = String()
        state.data = f'pending={pending} active={active}'
        self.state_pub.publish(state)

        if self.publish_debug:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(result.plot(), 'bgr8'))

        if best is None:
            return  # 没有待激活靶 -> 不发位姿，下游自动停控

        _, u, v, w_px = best
        # 框中心低通，抑制逐帧抖动
        if self._uv is None:
            self._uv = (u, v)
        else:
            a = self.center_alpha
            self._uv = (self._uv[0] + a * (u - self._uv[0]),
                        self._uv[1] + a * (v - self._uv[1]))
        u, v = self._uv

        # 距离：优先用固定标称距离（本仿真机关位置固定），否则用框宽估算
        if self.nominal_d > 0.0:
            distance = self.nominal_d
        else:
            distance = self.fx * self.plate_w / max(w_px, 1.0)

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
