"""一键启动蓝胖鱼算法链：检测（YOLO 或 HSV） + 瞄准/开火（+ 可选相机显示）。

用法：
    ros2 launch energy_vision energy_vision.launch.py
    ros2 launch energy_vision energy_vision.launch.py use_hsv:=true          # 用 HSV 方案
    ros2 launch energy_vision energy_vision.launch.py fire_mode:=2           # 对准才射
    ros2 launch energy_vision energy_vision.launch.py show_camera:=true      # 另开显示窗口
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_yolo = LaunchConfiguration('use_yolo')
    use_hsv = LaunchConfiguration('use_hsv')
    show_camera = LaunchConfiguration('show_camera')
    fire_mode = LaunchConfiguration('fire_mode')
    model = LaunchConfiguration('model')
    conf = LaunchConfiguration('conf')

    return LaunchDescription([
        DeclareLaunchArgument('use_yolo', default_value='true',
                              description='用 YOLO 检测（默认，推荐）'),
        DeclareLaunchArgument('use_hsv', default_value='false',
                              description='用 HSV 灯条检测（与 use_yolo 二选一）'),
        DeclareLaunchArgument('show_camera', default_value='false',
                              description='是否启动 camera_node 显示窗口'),
        DeclareLaunchArgument('fire_mode', default_value='1',
                              description='0=只跟不射 1=识别即射 2=对准才射'),
        DeclareLaunchArgument('model', default_value='',
                              description='YOLO 权重路径，留空用包内 models/rune_best.pt'),
        DeclareLaunchArgument('conf', default_value='0.30'),

        Node(
            package='energy_vision',
            executable='yolo_target_node',
            name='yolo_target_node',
            condition=IfCondition(use_yolo),
            parameters=[{'model': model, 'conf': conf}],
            output='screen',
        ),
        Node(
            package='energy_vision',
            executable='light_detector',
            name='energy_light_detector',
            condition=IfCondition(use_hsv),
            output='screen',
        ),
        Node(
            package='energy_vision',
            executable='pose_aim_node',
            name='pose_aim_node',
            parameters=[{'fire_mode': fire_mode}],
            output='screen',
        ),
        Node(
            package='energy_vision',
            executable='camera_node',
            name='energy_camera_node',
            condition=IfCondition(show_camera),
            output='screen',
        ),
    ])
