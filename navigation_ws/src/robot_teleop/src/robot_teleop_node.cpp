#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> //для передлачи команд управления роботом
#include <libevdev/libevdev.h> // для работы с устройством ввода
#include <fcntl.h> //для работы с файловами дескрипторами
#include <unistd.h> // для POSIX операционнных системных вызовов


using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    // uint8_t key_pressed = 1;
    uint8_t key_states[256] ;
    KeyboardTeleop()
        :Node("keyboard_teleop")
    {
        // Инициализация
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); // публикатор

        // const char *device ="/dev/input/by-path/platform-i8042-serio-0-event-kbd"; // Встроенная клава
        const char *device ="/dev/input/by-path/pci-0000:00:14.0-usb-0:1:1.0-event-kbd"; // домашняя клава

        fd = open(device,O_RDONLY | O_NONBLOCK); // окрываем в режиме чтения и не блокирующем режиме
        if(fd  < 0){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Не удалось открыть устройство");
            return;
        }

        rc = libevdev_new_from_fd(fd,&dev);
        if(rc < 0){
            RCLCPP_ERROR_STREAM(this->get_logger(), "failed to init libevdev");
            return;
        }

        timer = create_wall_timer(20ms,std::bind(&KeyboardTeleop::timer_callback,this));
        RCLCPP_INFO(this->get_logger(), "Используйте W A S D для управления");
    }

    // Деструкторрр класса
    ~KeyboardTeleop(){
        libevdev_free(dev);
        close(fd);
    }

    // функция для регистрации команд
    void timer_callback(){
        struct input_event ev;
        // считывание события
        rc = libevdev_next_event(dev,LIBEVDEV_READ_FLAG_NORMAL,&ev);
        if(rc == LIBEVDEV_READ_STATUS_SUCCESS){ // если событие считано
            if(ev.type == EV_KEY){ // событие "нажата клавиша"
                key_states[ev.code] = ev.value;


                // switch(ev.code){
                // case 17:
                //     RCLCPP_INFO_STREAM(this->get_logger(), "FORWARD");
                //     msg.linear.x = 1;
                //     break;
                // case 31:
                //     RCLCPP_INFO_STREAM(this->get_logger(), "BACKWARD");
                //     msg.linear.x = -1;
                //     break;
                // case 30:
                //     RCLCPP_INFO_STREAM(this->get_logger(), "LEFT");
                //     msg.angular.z = 1;
                //     break;
                // case 32:
                //     RCLCPP_INFO_STREAM(this->get_logger(), "RIGHT");
                //     msg.angular.z = -1;
                //     break;
                // }

                switch(ev.value){
                case 0:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " отпущена");
                    msg.linear.x = 0;
                    msg.angular.z = 0;
                    // key_pressed = 0;
                    break;
                case 1:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " нажата");
                    // key_pressed = 1;
                    break;
                case 2:
                    RCLCPP_INFO_STREAM(this->get_logger(), "клавиша " << ev.code << " удерживается");
                    break;
                }

            }

        }

        // w = 17
        // a = 30
        // s = 31
        // d = 32
        // RCLCPP_INFO_STREAM(this->get_logger(), "key_pressed " << (int)key_pressed);

        update_velocity();
        // Публикуем сообщение
        cmd_pub->publish(msg);


    }

    void update_velocity() {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        // Линейное движение (W/S)
        if (key_states[17] > 0) msg.linear.x = 0.25;   // W - вперед
        if (key_states[31] > 0) msg.linear.x = -0.25;  // S - назад

        // Поворот (A/D)
        if (key_states[30] > 0) msg.angular.z = 0.5;  // A - поворот налево
        if (key_states[32] > 0) msg.angular.z = -0.5; // D - поворот направо

        // Комбинации работают автоматически!
        // W + A = вперед + поворот = движение по дуге
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer;
    int fd = -1; // файловый дескриптор для устройства
    struct libevdev *dev = NULL; // структура для работы с устройством через libevdev
    int rc; // Идентификатор libevdev c открытым файловым дескриптором
    geometry_msgs::msg::Twist msg;


};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;


}


