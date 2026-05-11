from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.1',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link', # link робота
            '--child-frame-id', 'lidar_link'] # link лидара
        ),
        Node(
            package='robot_odometry',
            executable='robot_odometry_node',
            output='screen',
            name='robot_odometry_node'
        ),
        Node(
            package='robot_slam',
            executable='robot_slam_node',
            output='screen',
            name='robot_slam_node',
            parameters=[{
                            # Шумы фильтра Калмана
                            'motion_noise': [0.01, 0.01, 0.001],
                            'measurement_noise': [0.05, 0.05, 0.01],

                            # Параметры ICP
                            'MaximumIterations': 200,
                            'EuclideanFitnessEpsilon': 1e-6,
                            'MaxCorrespondenceDistance': 0.3,

                            # Параметры фильтрации облака точек
                            'Leaf': 0.04,
                            'MeanK': 30,
                            'StddevMulThresh': 1.5
                        }]
        ),
        Node(
            package="global_map_ex", # Указываем пакет
            executable="global_map_ex_node", # Исполняемый файл
            name="global_map_ex_node_1", # Имя для узла, может помочь при запуске одного и того же узла одновременно
            output="screen", # Выводить консольную информацию на экран
            parameters=[ # Сами параметры
                {"map_width": 1500,
                "map_height": 1500,
                "map_resolution": 0.07}
                ]
            ),
        Node(
            package="robot_teleop", # Указываем пакет
            executable="robot_teleop_node", # Исполняемый файл
            name="robot_teleop_node_2", # Имя для узла, может помочь при запуске одного и того же узла одновременно
            output="screen", # Выводить консольную информацию на экран
            parameters=[ # Сами параметры
                ]
            )
    ])
















