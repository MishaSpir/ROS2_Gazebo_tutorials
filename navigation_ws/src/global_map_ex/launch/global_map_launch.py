from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): # Функция возвращает конфигурацию запуска, в которую входит один узел
    return LaunchDescription([
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
            )
    ])
