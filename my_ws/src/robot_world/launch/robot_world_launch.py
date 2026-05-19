import os
from ament_index_python.packages import get_package_share_directory
from launch import  LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Полный путь к файлу модели мира
    world_path = '/home/misha/building_editor_models/test_world'
    # Полный путь к файлу модели робота
    robot_path = '/home/misha/model_editor_models/my_robot/model.sdf'

    dynamic_obstacle_path = '/home/misha/model_editor_models/moving_block/model.sdf'


    # Перевод в xml
    robot_xml = open(robot_path,'r').read()
    robot_xml = robot_xml.replace('"','\\"')

    dyn_obs_xml = open(dynamic_obstacle_path,'r').read()
    dyn_obs_xml = dyn_obs_xml.replace('"','\\"')

    # Параметры спавна (координаты и угол)
    spawn_x = 0.0   # метры
    spawn_y = 0.0   # метры  
    spawn_z = 0.0   # метры
    spawn_yaw = 1.0  # по z = - 1 (-90 градусов)

    obs_spawn_x = 0.0   # метры
    obs_spawn_y = 1.5   # метры  
    obs_spawn_z = 0.0   # метры
    obs_spawn_yaw = 0.0  
    
    
    # Добавляем pose в аргументы спавна
    robot_spawn_args = f'{{name: "robot_vac", xml: "{robot_xml}", initial_pose: {{position: {{x: {spawn_x}, y: {spawn_y}, z: {spawn_z}}}, orientation: {{z: {spawn_yaw}}}}}}}'

    dyn_obs_spawn_args = f'{{name: "moving_block", xml: "{dyn_obs_xml}", initial_pose: {{position: {{x: {obs_spawn_x}, y: {obs_spawn_y}, z: {obs_spawn_z}}}, orientation: {{z: {obs_spawn_yaw}}}}}}}'
    # robot_spawn_args = '{name: \"robot_vac\", xml: \"' + robot_xml + '\"}'


    # Параметры платформы (можно менять здесь)
    platform_params = {
        'wheel_radius': 0.05,
        'wheel_width': 0.02,
        'wheel_base': 0.26,
        'ticks_per_revolution': 400.0
    }

    # Нода одометрии
    # odometry_node = Node(
    #     package='robot_odometry',
    #     executable='robot_odometry_node',
    #     name='robot_odometry_node',
    #     output='screen',
    #     parameters=[platform_params]  # Передаем словарь с параметрами
    # )

    # Нода управления
    keyboard_node = Node(
        package='robot_teleop',
        executable='robot_teleop_node',
        name='robot_teleop_node',
        output='screen',
        parameters=[ 
                {"linear_speed": 1.0,
                "angular_speed": 1.0,
                "radar_ang_speed": 0.5,
                "radar_timer_period_ms": 50,
                "radar_min_angle": -1.5,
                "radar_max_angle": 1.5           
                }
                ]
    )

    return LaunchDescription([
        Node(
            # Публикатор статической трансформации
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.3',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link', # link робота
            '--child-frame-id', 'radar_link'] # link лидара
        ),
        ExecuteProcess(
            cmd = ['gazebo','--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output = 'screen'
        ),
        TimerAction(
            period=1.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', robot_spawn_args],
                    output='screen'
                )
            ]
        ),
                # Спавн препятствия через 3 секунд (через 1,5 секунды после робота)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', dyn_obs_spawn_args],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=6.0,  
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'move_block', 'move_block.py'],
                    output='screen'
                )
            ]
        ),
        # odometry_node,
        keyboard_node

    ])








