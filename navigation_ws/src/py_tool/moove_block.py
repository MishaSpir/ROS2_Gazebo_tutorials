#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

def move_object():
    rospy.init_node('move_dynamic_object')
    
    # Дожидаемся запуска сервиса управления моделью в Gazebo
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        # Задаём новое положение объекта
        state_msg = ModelState()
        state_msg.model_name = 'moving_block'
        state_msg.pose = Pose(Point(x=1.0, y=0.0, z=0.25), Quaternion(0,0,0,1))
        state_msg.twist = Twist()  # Можно задать скорость, если нужно
        
        # Вызываем сервис
        resp = set_state(state_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_object()
    except rospy.ROSInterruptException:
        pass    