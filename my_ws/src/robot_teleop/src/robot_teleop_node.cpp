#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  // Добавляем для получения угла
#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <std_msgs/msg/int8.hpp>  // Для публикации направления

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    uint8_t key_states[256];
    
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        // Параметры
        this->declare_parameter("linear_speed", 1.0);
        this->declare_parameter("angular_speed", 1.0);
        this->declare_parameter("radar_ang_speed", 1.0);
        this->declare_parameter("radar_timer_period_ms", 50);
        this->declare_parameter("radar_min_angle", -1.745);   // -100 градусов
        this->declare_parameter("radar_max_angle", 1.745);    // +100 градусов
        
        int period_ms = this->get_parameter("radar_timer_period_ms").as_int();
        auto radar_timer_period = std::chrono::milliseconds(period_ms);
        
        linear_speed = get_parameter("linear_speed").as_double();
        angular_speed = get_parameter("angular_speed").as_double();
        radar_ang_speed = get_parameter("radar_ang_speed").as_double();
        radar_min_angle = get_parameter("radar_min_angle").as_double();
        radar_max_angle = get_parameter("radar_max_angle").as_double();
        
        // Инициализация переменных радара
        radar_direction = 1;  // 1 = вперед, -1 = назад
        current_radar_angle = 0.0;
        is_stopped = false;
        
        // Обнуляем сообщение для радара
        radar_msg.angular.x = 0;
        radar_msg.angular.y = 0;
        radar_msg.angular.z = 0;
        radar_msg.linear.x = 0;
        radar_msg.linear.y = 0;
        radar_msg.linear.z = 0;
        
        // Инициализация публикаторов
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        radar_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/radar_cmd_vel", 10);
        
        // Подписка на состояние суставов для получения угла радара
        joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KeyboardTeleop::joint_callback, this, std::placeholders::_1));


        // Публикатор направления радара
        radar_direction_pub = this->create_publisher<std_msgs::msg::Int8>(
            "/radar_direction", 10);    
        
        // Инициализация клавиатуры
        const char *device = "/dev/input/by-path/pci-0000:00:14.0-usb-0:1:1.0-event-kbd";
        fd = open(device, O_RDONLY | O_NONBLOCK);
        if(fd < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Не удалось открыть устройство");
            return;
        }
        
        rc = libevdev_new_from_fd(fd, &dev);
        if(rc < 0) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "failed to init libevdev");
            return;
        }
        
        // Таймеры
        timer = create_wall_timer(20ms, std::bind(&KeyboardTeleop::timer_callback, this));
        radar_tick_timer = create_wall_timer(radar_timer_period, std::bind(&KeyboardTeleop::radar_tick_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), " W A S D для управления");
        RCLCPP_INFO(this->get_logger(), "Радар сканирует от %.1f до %.1f градусов", 
                   radar_min_angle * 180.0 / M_PI, radar_max_angle * 180.0 / M_PI);
    }
    
    ~KeyboardTeleop() {
        libevdev_free(dev);
        close(fd);
    }
    
    // Нормализация угла в диапазон [-π, π)
    double normalizeAngle(double angle) {
        angle = fmod(angle, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        if (angle >= M_PI) angle -= 2 * M_PI;
        return angle;
    }
    
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "radar_joint") {
                current_radar_angle = normalizeAngle(msg->position[i]);
                
                // Проверяем границы и меняем направление
                if (current_radar_angle >= radar_max_angle && radar_direction > 0 && !is_stopped) {     
                    radar_direction = -1;

                    auto direction_msg = std_msgs::msg::Int8();
                    direction_msg.data = radar_direction;
                    radar_direction_pub->publish(direction_msg);
                    // Делаем паузу перед сменой направления
                    is_stopped = true;
                    stop_timer = this->create_wall_timer(
                        100ms, std::bind(&KeyboardTeleop::resume_rotation, this));
                }
                else if (current_radar_angle <= radar_min_angle && radar_direction < 0 && !is_stopped) {
                    radar_direction = 1;
                    auto direction_msg = std_msgs::msg::Int8();
                    direction_msg.data = radar_direction;
                    radar_direction_pub->publish(direction_msg);
                    // Делаем паузу перед сменой направления
                    is_stopped = true;
                    stop_timer = this->create_wall_timer(
                        100ms, std::bind(&KeyboardTeleop::resume_rotation, this));
                }
                break;
            }
        }
    }
    
    void resume_rotation() {
        is_stopped = false;
        stop_timer.reset();  // Останавливаем таймер
        // RCLCPP_INFO(this->get_logger(), "Возобновляем вращение радара");
    }
    
    void timer_callback() {
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if(rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            if(ev.type == EV_KEY) {
                key_states[ev.code] = ev.value;
                
                switch(ev.value) {
                case 0:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " отпущена");
                    msg.linear.x = 0;
                    msg.angular.z = 0;
                    break;
                case 1:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " нажата");
                    break;
                case 2:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " удерживается");
                    break;
                }
            }
        }
        
        update_velocity();
        cmd_pub->publish(msg);
    }
    
    void radar_tick_timer_callback() {
        if (!is_stopped) {
            // Вращаем радар в текущем направлении
            radar_msg.angular.z = radar_ang_speed * radar_direction;
        } else {
            // Радар остановлен
            radar_msg.angular.z = 0;
        }
        
        radar_cmd_pub->publish(radar_msg);
    }
    
    void update_velocity() {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        
        if (key_states[17] > 0) msg.linear.x = linear_speed;   // W
        if (key_states[31] > 0) msg.linear.x = -linear_speed;  // S
        if (key_states[30] > 0) msg.angular.z = angular_speed;  // A
        if (key_states[32] > 0) msg.angular.z = -angular_speed; // D
    }
    
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr radar_cmd_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr radar_direction_pub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr radar_tick_timer;
    rclcpp::TimerBase::SharedPtr stop_timer;
    
    int fd = -1;
    struct libevdev *dev = NULL;
    int rc;
    
    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Twist radar_msg;
    
    double linear_speed;
    double angular_speed;
    double radar_ang_speed;
    double radar_min_angle;
    double radar_max_angle;
    
    double radar_direction;
    double current_radar_angle;
    bool is_stopped;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}