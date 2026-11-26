#!/usr/bin/env python3
"""从 /camera/image_raw 按间隔抽帧，用于 YOLO 标注。

用法（仿真跑着、云台在转的时候执行，多跑几个阶段/角度）：
    ros2 run energy_vision collect_frames --ros-args -p out_dir:=yolo_frames -p every:=10

建议：跑 fixed 时让靶停在屏幕不同位置各采一轮；跑 small/big 时扫一圈。
每类目标（pending / active）尽量各 150~300 张，覆盖不同俯仰/偏航/距离。
"""
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCollector(Node):
    def __init__(self):
        super().__init__('frame_collector')
        self.declare_parameter('out_dir', 'yolo_frames')
        self.declare_parameter('every', 10)
        self.declare_parameter('jpeg_quality', 92)
        self.out_dir = self.get_parameter('out_dir').value
        self.every = max(1, int(self.get_parameter('every').value))
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frames = 0
        self.saved = 0
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.get_logger().info(f'抽帧: 每 {self.every} 帧存一张 -> {self.out_dir}/')

    def on_image(self, msg):
        self.frames += 1
        if self.frames % self.every != 0:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        path = os.path.join(self.out_dir, f'frame_{self.saved:05d}.jpg')
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        self.saved += 1
        if self.saved % 20 == 0:
            self.get_logger().info(f'已保存 {self.saved} 张')


def main():
    rclpy.init()
    node = FrameCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'停止，共保存 {node.saved} 张')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
