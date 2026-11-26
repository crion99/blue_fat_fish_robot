#!/usr/bin/env python3
"""用训练好的 YOLO 检测待激活靶板，输出与 light_detector 完全相同的位姿话题，
   下游 pose_aim_node / 瞄准链一行都不用改。

输出：
    /vision/pending_target_pose  geometry_msgs/PoseStamped
        position = 靶心在相机光轴系下的坐标（x 右, y 下, z 前，单位 m）
    /vision/rune_states          std_msgs/String           "pending=N active=M"

原理（单目、只要瞄准角，不需要 6DOF PnP）：
    - 框中心 (u,v) -> 相对光轴的角度：rel_yaw = -atan2(u-cx, fx), rel_pitch = -atan2(v-cy, fy)
    - 框宽 w_px -> 距离：d = fx * plate_width_m / w_px
    - 该点即目标在相机系的位置：x = (u-cx)/fx*d, y = (v-cy)/fy*d, z = d
    pose_aim_node 会自动再做 相机->枪口、重力补偿、提前量。

用法：
    ros2 run energy_vision yolo_target_node --ros-args -p model:=yolo/runs/rune/weights/best.pt
"""
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from ultralytics import YOLO

PENDING = 0
ACTIVE = 1


class YoloTargetNode(Node):
    def __init__(self):
        super().__init__('yolo_target_node')
        self.declare_parameter('model', 'yolo/runs/rune/weights/best.pt')
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('plate_width_m', 0.195)  # 靶板真实宽度（m）
        self.declare_parameter('publish_debug_image', True)

        self.model = YOLO(self.get_parameter('model').value)
        self.conf = float(self.get_parameter('conf').value)
        self.plate_w = float(self.get_parameter('plate_width_m').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = 0.0

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/pending_target_pose', 10)
        self.state_pub = self.create_publisher(String, '/vision/rune_states', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/yolo_debug', 10)
        self.get_logger().info('yolo_target_node 启动')

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
        best = None  # (score, u, v, w)
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            w = x2 - x1
            if cls == PENDING:
                pending += 1
                if best is None or conf > best[0]:
                    best = (conf, 0.5 * (x1 + x2), 0.5 * (y1 + y2), w)
            elif cls == ACTIVE:
                active += 1

        state = String()
        state.data = f'pending={pending} active={active}'
        self.state_pub.publish(state)

        if self.publish_debug:
            import cv2  # 仅调试用
            annotated = result.plot()
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

        if best is None:
            return  # 没有待激活靶 -> 不发位姿，下游会自动停控

        _, u, v, w_px = best
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
