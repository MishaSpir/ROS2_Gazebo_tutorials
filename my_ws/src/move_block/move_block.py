#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
import time

class MoveObject(Node):
    def __init__(self):
        super().__init__('move_dynamic_object')
        self.get_logger().info('Move Object Node Started')
        
        # Создаем клиент для сервиса /set_entity_state
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Ждем пока сервис станет доступным
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис /set_entity_state недоступен, ждем...')
        
        self.get_logger().info('Сервис /set_entity_state доступен')

    def move_object(self, model_name, x, y, z):
        # Формируем запрос
        request = SetEntityState.Request()
        request.state.name = model_name
        
        # Задаем позицию
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        
        # Задаем ориентацию (без поворота)
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = 0.0
        request.state.pose.orientation.w = 1.0
        
        # Обнуляем скорость
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        
        # Вызываем сервис
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Объект {model_name} перемещен на ({x:.2f}, {y:.2f}, {z:.2f})')
            return True
        else:
            self.get_logger().error('Ошибка при вызове сервиса')
            return False

def main(args=None):
    rclpy.init(args=args)
    mover = MoveObject()
    
    # Параметры движения
    model_name = 'moving_block'
    start_x = -2.0
    end_x = 2.0
    step = 0.1  # шаг перемещения в метрах
    y_pos = 1.5
    z_pos = 0.25
    delay = 0.1  # задержка между перемещениями (секунды)
    
    current_x = start_x
    direction = 1  # 1 = вперед, -1 = назад
    
    mover.get_logger().info(f'Начинаем движение объекта от {start_x} до {end_x}')
    
    try:
        while rclpy.ok():
            # Перемещаем объект
            mover.move_object(model_name, current_x, y_pos, z_pos)
            
            # Обновляем позицию
            current_x += step * direction
            
            # Проверяем границы
            if current_x >= end_x:
                current_x = end_x
                direction = -1
                mover.get_logger().info('Достигнут конец, двигаемся обратно...')
            elif current_x <= start_x:
                current_x = start_x
                direction = 1
                mover.get_logger().info('Достигнуто начало, двигаемся вперед...')
            
            # Задержка
            time.sleep(delay)
            
    except KeyboardInterrupt:
        mover.get_logger().info('Движение остановлено пользователем')
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()