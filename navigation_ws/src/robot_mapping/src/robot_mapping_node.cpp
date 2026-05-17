#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class RobotCostmap : public rclcpp::Node
{
public:
    RobotCostmap()
        : Node("robot_odometry_node")
    {
        // Параметры карты
        costmap.header.frame_id = "map";
        costmap.info.resolution = 0.1; // Метры в ячейке
        costmap.info.width = 200; // Ширина в ячейках
        costmap.info.height = 200; // Высота в ячейках

        // ПОложение начала карты
        costmap.info.origin.position.x = -(costmap.info.width/2.0) * costmap.info.resolution;
        costmap.info.origin.position.y = -(costmap.info.height/2.0) * costmap.info.resolution;
        costmap.data.resize(costmap.info.width * costmap.info.height);


        subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RobotCostmap::scanCallback,
                      this,
                      std::placeholders::_1));

        publisher =
            this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
        timer = this->create_wall_timer(100ms,
                                        std::bind(&RobotCostmap::timer_callback,
                                                  this));
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;

    nav_msgs::msg::OccupancyGrid costmap;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        for(unsigned int i =0; i < costmap.data.size(); i++){
            costmap.data.at(i) = 0;
        }

        float angle;
        float distance;

        // Доступ к точкам облака
        for (int i = 0; i < msg->ranges.size(); ++i) {
            distance = msg->ranges[i];

            if (distance < msg->range_min || distance > msg->range_max) {
                continue;
            }

            angle = msg->angle_min + i * msg->angle_increment;

            // Координаты ячейки занятой препятствием (в метрах)

            float x = distance * cos(angle);
            float y = distance * sin(angle);

            int x_cell, y_cell;

            cellsFromCoordinates(x, y, x_cell, y_cell);

            // Радиус окретсности препятствия для анализа стоимости. (Чем меньше, тем меньше выислений)
            int radius = 1.0 / costmap.info.resolution;

            //Costmap
            for(int x_c = x_cell-radius; x_c <= x_cell + radius; x_c++){
                for(int y_c = y_cell-radius; y_c <= y_cell+radius; y_c++){
                    float d = sqrt(pow(x_cell - x_c,2) + pow(y_cell - y_c,2)) * costmap.info.resolution;
                    int cost = computeCost(d);
                    int index = indexFromCells(x_c, y_c);
                    if (index > 0){
                        if(costmap.data.at(index) < cost && d < 1.0){
                            costmap.data.at(index) = cost;
                        }
                    }
                }
            }

            // costmap.header.stamp = this->now();
            // publisher->publish(costmap);
        }
    }



    void timer_callback(){

        costmap.header.stamp = this->now();
        publisher->publish(costmap);
    }

    // Вычисление единого индекса в массие data
    unsigned int indexFromCells(int x_cell, int y_cell){\
        // Проверка находится ли координата внутри карты
        if(x_cell > 0 && y_cell > 0 && x_cell < costmap.info.width && y_cell < costmap.info.width) {
            unsigned int index = (y_cell * costmap.info.width + x_cell);
            return index;
        }
        else {
            return 0;
        }
    }

    // Перевод координат из метрических координат в координаты ячейки
    void cellsFromCoordinates(double x, double y, int &x_cell, int & y_cell){
        x_cell = (x - costmap.info.origin.position.x) / costmap.info.resolution;
        y_cell = (y - costmap.info.origin.position.y) / costmap.info.resolution;
    }

    // Вычисление стоимости ячейки
    int8_t computeCost(double distance) {
        double weight = 5;
        double inscribed_radius = 0.2;
        int8_t cost = 0;
        if (distance == 0.0) cost = 100; // LETHAL_OBSTACLE
        else if (distance <= inscribed_radius) cost = 99;
        else {
            double euclidean_distance = distance;
            double factor = exp(-1.0 * weight * (euclidean_distance - inscribed_radius));
            cost = (int8_t)((99-1)*factor);
        }
        return cost;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCostmap>());
    rclcpp::shutdown();
    return 0;
}







































