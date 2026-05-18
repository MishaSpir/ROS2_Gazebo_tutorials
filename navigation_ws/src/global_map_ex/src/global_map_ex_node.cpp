#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class GlobalMap : public rclcpp::Node {
public:
    GlobalMap() : Node("global_map_node") {
        // Объявление параметров
        this->declare_parameter("timer_period_ms", 50);
        int period_ms = this->get_parameter("timer_period_ms").as_int();
        auto period = std::chrono::milliseconds(period_ms);

        this->declare_parameter("map_width", 1500);
        this->declare_parameter("map_height", 1500);
        this->declare_parameter("map_resolution", 0.07);
        this->declare_parameter("filter_gain", 0.8);      // K = 0.8 (приоритет глобальной карте)
        
        // Получение параметров
        resolution = get_parameter("map_resolution").as_double();
        map_width = get_parameter("map_width").as_int();
        map_height = get_parameter("map_height").as_int();
        K = get_parameter("filter_gain").as_double();      // Коэффициент фильтра
        ekf_true = false;
        
        map_size = map_width * map_height;
        
        // Инициализация глобальной карты
        global_map.header.frame_id = "map";
        global_map.info.resolution = resolution;
        global_map.info.width = map_width;
        global_map.info.height = map_height;
        global_map.info.origin.position.x = -(map_width * resolution) / 2.0;
        global_map.info.origin.position.y = -(map_height * resolution) / 2.0;
        global_map.info.origin.position.z = 0.0;
        global_map.data.resize(map_size, -1);  // -1 = неизвестно
        
        // Подписчики
        scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&GlobalMap::scanCallback, this, std::placeholders::_1));
        
        pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ekf_pose", 10, std::bind(&GlobalMap::poseCallback, this, std::placeholders::_1));
        
        // Публикатор
        map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 10);
        // timer = this->create_wall_timer(period, std::bind(&GlobalMap::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Global Map Node initialized with K=%.2f", K);
    }

private:
    // Параметры
    double resolution;
    int map_width;
    int map_height;
    int map_size;
    double K;                      // Коэффициент комплементарного фильтра
    bool ekf_true;

    
    // Данные
    geometry_msgs::msg::PoseStamped current_pose;
    nav_msgs::msg::OccupancyGrid global_map;
    bool has_pose = false;
    
    // ROS коммуникации
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::TimerBase::SharedPtr timer;

    
    // Извлечение угла yaw из кватерниона
    double getYaw(const geometry_msgs::msg::Quaternion &q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double yaw, pitch, roll;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        current_pose = *pose;
        ekf_true = true;

    }
    
    
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        if(ekf_true == true){
            ekf_true = false;
            global_map.header.stamp = this->get_clock()->now();

        // Положение робота
        double robot_x = current_pose.pose.position.x;
        double robot_y = current_pose.pose.position.y;
        double robot_yaw = getYaw(current_pose.pose.orientation);

        float angle;
        float distance;

        // Доступ к точкам облака
        for (int i = 0; i < msg->ranges.size(); ++i) {
            distance = msg->ranges[i];

            if (distance < msg->range_min || distance > msg->range_max) {
                continue;
            }

            angle = msg->angle_min + i * msg->angle_increment;

            // Заполняем карту занятости ------------------------------------------------------------
            float n = distance / resolution; // Делим расстояние на количество ячеек, узнаём сколько ячеек заполнить нулями
            // RCLCPP_INFO_STREAM(this->get_logger(), "angle:"<<angle<<"  "<<"min_distance"<<"  "<<min_distance);

            float x_global;
            float y_global;

            for (int j = 0; j < n; j++){
                float iter_range = resolution * j;
                float x_i = iter_range * cos(angle);
                float y_i = iter_range * sin(angle);

                // Преобразование в глобальные координаты карты
                x_global = robot_x + std::cos(robot_yaw) * x_i - std::sin(robot_yaw) * y_i;
                y_global = robot_y + std::sin(robot_yaw) * x_i + std::cos(robot_yaw) * y_i;


                uint x1_cell = (x_global - global_map.info.origin.position.x)/global_map.info.resolution;
                uint y1_cell = (y_global - global_map.info.origin.position.y)/global_map.info.resolution;
                uint cell = y1_cell * global_map.info.width + x1_cell;

                if (cell >= map_size || (x1_cell >= map_width) || (y1_cell >= map_height)){
                    continue;
                }
                if (global_map.data.at(cell) == -1){
                    global_map.data.at(cell) = 0;
                }
                else{
                    global_map.data.at(cell) = global_map.data.at(cell) * K; // Комплиментарный фильтр

                }

            }
            float x_i = distance * cos(angle);
            float y_i = distance * sin(angle);

            // Преобразование в глобальные координаты карты
            x_global = robot_x + std::cos(robot_yaw) * x_i - std::sin(robot_yaw) * y_i;
            y_global = robot_y + std::sin(robot_yaw) * x_i + std::cos(robot_yaw) * y_i;

            uint x1_cell = (x_global - global_map.info.origin.position.x)/global_map.info.resolution;
            uint y1_cell = (y_global - global_map.info.origin.position.y)/global_map.info.resolution;
            uint cell = y1_cell * global_map.info.width + x1_cell;
            if (cell >= map_size || (x1_cell >= map_width) || (y1_cell >= map_height)){
                continue;
            }
            if (global_map.data.at(cell) == -1){
                global_map.data.at(cell) = 100;
            }
            else{
                if ((global_map.data.at(cell) * K + (1-K)) >= 100){
                    global_map.data.at(cell) = 100;
                }
                else{
                    global_map.data.at(cell) = global_map.data.at(cell) * K + (1-K)*100; // Комплиментарный фильтр
                }
            }
        }
                map_pub->publish(global_map);

        }
        

    }

    void timerCallback(){
        // map_pub->publish(global_map);
    }

};




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMap>());
    rclcpp::shutdown();
    return 0;
}