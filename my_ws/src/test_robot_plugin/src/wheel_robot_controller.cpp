#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> // для скорости колёс
#include <sensor_msgs/msg/joint_state.hpp> // для иметации обратной связи с колёс

using namespace std::chrono_literals;

namespace gazebo_plugins {
class WheelVelocityController : public gazebo::ModelPlugin{
public:
    WheelVelocityController() :ModelPlugin(){}
    void Load (gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override{
        // Инициализация ROS 2 узла, который взаимод с РОС в рамках плагина газебо
        node = gazebo_ros::Node::Get(sdf);

        joint_right = model->GetJoint("right_wheel_joint");//указатели на объекты
        joint_left = model->GetJoint("left_wheel_joint");
        joint_radar = model->GetJoint("radar_joint");

        if(!joint_right || !joint_left || !joint_radar ){
            RCLCPP_ERROR(node->get_logger(), "Some joints not found");
            return;
        }

        // Подписываемя на топик geometry_msgs/Twist для управления скоростью
        cmd_vel_sub = node-> create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",10,std::bind(&WheelVelocityController::cmd_vel_callback,this,std::placeholders::_1));

        radar_cmd_vel_sub = node-> create_subscription<geometry_msgs::msg::Twist>(
            "/radar_cmd_vel",10,std::bind(&WheelVelocityController::radar_cmd_vel_callback,this,std::placeholders::_1));    

        // Создаем публикатор для публикации углов поворота колёс
        joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);

        // Настройка таймера для публикации углов поворота
        timer = node-> create_wall_timer(100ms,std::bind(&WheelVelocityController::timer_callback,this));
    }
private:
    // ROS 2 узел
    gazebo_ros::Node::SharedPtr node;

    // Сочленения колёс указатели на объекты
    gazebo::physics::JointPtr joint_right, joint_left,joint_radar;

    // Подписчик на команды скорости
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub,radar_cmd_vel_sub;

    // Публикатор для узлов сочленений
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

    // Таймер для переодической публикации
    rclcpp::TimerBase::SharedPtr timer;




    // эта функция вызывается при приходе новых данных о движении робота
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg){
        double linear_velocity = msg.linear.x; // Линейная скорость (м/с)
        double angular_velocity = msg.angular.z; // Угловая скорость (рад/с )
        


        // Вычиссляем скорости для левого и рпавого колёс
        double wheel_base = 0.26; //Расстояние между колёсам, метры
        double wheel_radius = 0.05; //Радиус колёс, метры


        // Тут важны знаки!! Для того чтобы робот ехал вперед, правое колесо
        // должно крутиться вперед (полож угловая скорость angular_velocity)
        // левое - назад (отр угловая скорость angular_velocity)
        double v_right = (linear_velocity + (angular_velocity * wheel_base / 2.0))/ wheel_radius;
        double v_left = (linear_velocity - (angular_velocity * wheel_base / 2.0))/ wheel_radius;



        // Устанавливаем скорости для правого и левого колёс
        if (joint_right){
            joint_right ->SetParam("fmax",0,100.0); //мааксимальный крутящий момент
            joint_right ->SetParam("vel",0,v_right); // желаемая угловая скорость [рад/с]
        }
        if (joint_left){
            joint_left ->SetParam("fmax",0,100.0);
            joint_left ->SetParam("vel",0,v_left);
        }
        

    }

    void radar_cmd_vel_callback(const geometry_msgs::msg::Twist& msg){
        double radar_ang_vel = msg.angular.z; // Угловая скорость (рад/с )
        double v_radar = radar_ang_vel * 0.05;
        if(joint_radar){
           joint_radar ->SetParam("fmax",0,100.0);
           joint_radar ->SetParam("vel",0,radar_ang_vel);
        }

    }

    void timer_callback(){
        if(joint_right && joint_left && joint_radar){
            //Создаем сообщение JointState
            sensor_msgs::msg::JointState joint_state_msg;

            // Утсанавливаем имена
            // имя может быть другое, но лучше то же , что и в sdf
            joint_state_msg.name.push_back("right_wheel_joint");
            joint_state_msg.name.push_back("left_wheel_joint");
            joint_state_msg.name.push_back("radar_joint");

            // Добавляем текущие углы поворота колёс
            joint_state_msg.position.push_back(joint_right->Position(0)); // Угол левого колса
            joint_state_msg.position.push_back(joint_left->Position(0));  // Угол правого колеса
            joint_state_msg.position.push_back(joint_radar->Position(0));  // Угол радара
            
            //Добавляем скорости вращения колёс
            joint_state_msg.velocity.push_back(joint_right->GetVelocity(0)); // Скорость правого колеса
            joint_state_msg.velocity.push_back(joint_left->GetVelocity(0));  // Скорость левогоо колеса
            joint_state_msg.velocity.push_back(joint_radar->GetVelocity(0));  // Скорость радара

            // Публикуем сообщения
            joint_states_pub -> publish(joint_state_msg);

        }

    }

};
}

// Регистрация плагина
GZ_REGISTER_MODEL_PLUGIN(gazebo_plugins::WheelVelocityController);
