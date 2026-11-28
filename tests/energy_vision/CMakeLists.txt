cmake_minimum_required(VERSION 3.8)

project(energy_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robomini_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Camera display node
add_executable(camera_node
    src/camera_node.cpp
)

ament_target_dependencies(camera_node
    rclcpp
    sensor_msgs
    cv_bridge
)

target_link_libraries(camera_node
    ${OpenCV_LIBS}
)

# Energy light detector
add_executable(light_detector
    src/light_detector.cpp
)

ament_target_dependencies(light_detector
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    robomini_interfaces
    tf2
    cv_bridge
)

target_link_libraries(light_detector
    ${OpenCV_LIBS}
)

add_executable(pose_feedback_diagnostic src/pose_feedback_diagnostic.cpp)
ament_target_dependencies(pose_feedback_diagnostic
    rclcpp
    geometry_msgs
    robomini_interfaces
)

add_executable(joint_state_adapter src/joint_state_adapter.cpp)
target_include_directories(joint_state_adapter PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../../gz_plugins/src")
ament_target_dependencies(joint_state_adapter rclcpp sensor_msgs std_msgs)

# Aiming: target -> yaw/pitch
add_executable(aim_node
    src/aim_node.cpp
)

target_include_directories(aim_node PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(aim_node
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

target_link_libraries(aim_node
    opencv_core
)

add_executable(aim_follower src/aim_follower.cpp)
target_include_directories(aim_follower PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../../gz_plugins/src")
ament_target_dependencies(aim_follower rclcpp std_msgs)

# Aiming: 靶心位姿 -> 瞄准转角（第三部分，对接 /vision_to_gimbal）
add_executable(pose_aim_node src/pose_aim_node.cpp)
ament_target_dependencies(pose_aim_node
    rclcpp
    geometry_msgs
    robomini_interfaces
)

# Install executables
install(
    TARGETS
        camera_node
        light_detector
        pose_feedback_diagnostic
        joint_state_adapter
        aim_node
        aim_follower
        pose_aim_node
    DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
    add_executable(pose_feedback_checks_test test/pose_feedback_checks_test.cpp)
    target_include_directories(pose_feedback_checks_test PRIVATE src)
    add_test(NAME pose_feedback_checks COMMAND pose_feedback_checks_test)

    add_executable(two_axis_geometry_test test/two_axis_geometry_test.cpp)
    target_include_directories(two_axis_geometry_test PRIVATE src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(two_axis_geometry_test PRIVATE opencv_core)
    add_test(NAME two_axis_geometry COMMAND two_axis_geometry_test)

    add_executable(aim_at_target_test test/aim_at_target_test.cpp)
    target_include_directories(aim_at_target_test PRIVATE src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(aim_at_target_test PRIVATE opencv_core)
    add_test(NAME aim_at_target COMMAND aim_at_target_test)
endif()

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
# 包内自带的 YOLO 权重（yolo_target_node 默认加载）
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/collect_frames.py DESTINATION lib/${PROJECT_NAME} RENAME collect_frames)
install(PROGRAMS scripts/train_yolo.py DESTINATION lib/${PROJECT_NAME} RENAME train_yolo)
install(PROGRAMS scripts/yolo_target_node.py DESTINATION lib/${PROJECT_NAME} RENAME yolo_target_node)

ament_package()
