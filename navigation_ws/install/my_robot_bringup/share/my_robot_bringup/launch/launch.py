from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Параметры платформы (можно менять здесь)
    platform_params = {
        'wheel_radius': 0.05,
        'wheel_width': 0.02,
        'wheel_base': 0.26,
        'ticks_per_revolution': 400.0
    }

    # Нода одометрии
    odometry_node = Node(
        package='robot_odometry',
        executable='robot_odometry_node',
        name='robot_odometry_node',
        output='screen',
        parameters=[platform_params]  # Передаем словарь с параметрами
    )

    # Нода управления
    keyboard_node = Node(
        package='robot_teleop',
        executable='robot_teleop_node',
        name='robot_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        odometry_node,
        keyboard_node
    ])
