#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

struct PlannerNode {
    int x, y; // Координаты ячейки
    double g_cost; // Стоимость до начала пути
    double h_cost; // Эвристическая стоимость до цели
    double f_cost; // Суммарная стоимость (g_host + h_host)
    int parent_x, parent_y; // Координаты родителя

    PlannerNode(int x, int y, double g_cost, double h_cost, int parent_x = -1, int parent_y = -1) :
        x(x),
        y(y),
        g_cost(g_cost),
        h_cost(h_cost),
        f_cost(g_cost + h_cost),
        parent_x(parent_x),
        parent_y(parent_y) {}

    bool operator<(const PlannerNode& other) const {
        return f_cost > other.f_cost;
    }

    bool operator==(const PlannerNode& other) const {
        return (x == other.x && y == other.y);
    }
};



class RobotPlanner : public rclcpp::Node
{
public:
    RobotPlanner()
        : Node("robot_odometry_node")
    {
        // Подписка на карту стоимости
        costmap_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap", 10, std::bind(&RobotPlanner::costmapCallback, this, std::placeholders::_1));

        // Подписка на целевую точку, до которой мы будем планировать траекторию
        goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&RobotPlanner::goalCallback, this, std::placeholders::_1));

        ekf_pose_sub = this->create_subscription
                        <geometry_msgs::msg::PoseStamped>(
                                        "/ekf_pose",
                                        10,
                                        std::bind(&RobotPlanner::ekfPoseCallback, this,
                                        std::placeholders::_1));

        publisher =
            this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }


private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;

    nav_msgs::msg::OccupancyGrid costmap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  ekf_pose_sub;

    // Информация о позиции робота
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose;
    bool has_pose = false;

    void ekfPoseCallback(const
                         geometry_msgs::msg::PoseStamped::SharedPtr msg){
        current_pose = msg;
        has_pose = true;
    }


    void costmapCallback(const nav_msgs::msg::OccupancyGrid &msg){
        costmap = msg;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        if (costmap.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Costmap not received yet!");
            return;
        }
        if (!has_pose){
            RCLCPP_WARN(this->get_logger(), "Ekf pose not received yet!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "goal start");

        geometry_msgs::msg::PoseStamped::SharedPtr goal_pose = msg;

        // Получаем текущее положение
        // Позиция
        double x = current_pose->pose.position.x;
        double y = current_pose->pose.position.y;


        // double x = 1.0;
        // double y = 1.5;

        int start_x, start_y;
        worldToMap(x, y, start_x, start_y);

        // ПОлучаем целевое положение из сообщения от RViz
        int goal_x, goal_y;
        worldToMap(goal_pose->pose.position.x, goal_pose->pose.position.y, goal_x, goal_y);

        // ПРоверяем валидность начальной и конечной точек
        if (!isCellValid(start_x, start_y)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Start cell is invalid!");
            return;
        }
        else if (!isCellValid(goal_x, goal_y)){
            RCLCPP_WARN(this->get_logger(), "Goal cell is invalid!");
            return;
        }

        if (isCellOccupied(start_x, start_y)){
            RCLCPP_INFO_STREAM(this->get_logger(), "Start cell is occupied!"<<start_x<<" "<<start_y);
            return;
        }
        else if (isCellOccupied(goal_x, goal_y)){
            RCLCPP_WARN(this->get_logger(), "Goal cell is occupied!");
            return;
        }

        // Вызываем A*
        std::vector<geometry_msgs::msg::PoseStamped> path = aStar(start_x, start_y, goal_x, goal_y);

        // Публикуем путь (если он найден)
        if (!path.empty()){
            std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = smoothPath(path);
            publishPath(smoothed_path);
        } else {
            RCLCPP_INFO(this->get_logger(), "No path found");
        }
    }

    // Функция для преобразования координат из метров в ячейки
    void worldToMap(double wx, double wy, int& mx, int& my){

        mx = static_cast<int>((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = static_cast<int>((wy - costmap.info.origin.position.y) / costmap.info.resolution);
    }

    // Функция для преобразования коорднат ячеек в метры (мировые координаты)
    void mapToWorld(int mx, int my, double& wx, double& wy){
        wx = costmap.info.origin.position.x + mx * costmap.info.resolution;
        wy = costmap.info.origin.position.y + my * costmap.info.resolution;
    }

    // Проверка, находистя ли ячейка в пределах карты
    bool isCellValid(int x, int y){
        return (x >= 0 && x < costmap.info.width && y >= 0 && y < costmap.info.height);
    }

    // Проверка, занята ли ячейка (препятствие)
    bool isCellOccupied(int x, int y){
        int index = y * costmap.info.width + x;
        return costmap.data[index] >= 99;

    }

    // Эвристическая функция (Евклидово расстояние) - для вычисления расстояния
    double heuristic(int x1, int y1, int x2, int y2){
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    // Восстановление пути (от целевого узла к начальному)
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(PlannerNode goal_PlannerNode,
                                                                 std::vector<PlannerNode> & closed_set){
        std::vector<geometry_msgs::msg::PoseStamped> path;
        int current_x = goal_PlannerNode.x;
        int current_y = goal_PlannerNode.y;
        while (current_x != -1 && current_y != -1){
            geometry_msgs::msg::PoseStamped pose;
            double world_x, world_y;
            mapToWorld(current_x, current_y, world_x, world_y);
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0; // Путь обычно плоский
            pose.pose.orientation.w = 1.0; // Без вращения
            path.push_back(pose);
            // Поиск родителя в closed_set
            bool found  = false;
            for(unsigned int n = 0; n < closed_set.size(); n++){
                if(closed_set.at(n).x == current_x && closed_set.at(n).y == current_y){
                    current_x = closed_set.at(n).parent_x;
                    current_y = closed_set.at(n).parent_y;
                    found = true;
                    break;
                }
            }
            if (!found) break; // На случай ошибок
        }
        std::reverse(path.begin(), path.end()); // Разворачиваем путь, чтобы он шёл от начала к цели
        return path;
    }

    // Реализация алгоритма А*
    std::vector<geometry_msgs::msg::PoseStamped> aStar(int start_x, int start_y, int goal_x, int goal_y) { // Передаём координаты начального и конечного положения
        // Инициализация
        std::priority_queue<PlannerNode> open_set;
        std::vector<PlannerNode> closed_set;

        PlannerNode start_PlannerNode(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));

        open_set.push(start_PlannerNode);
        closed_set.push_back(start_PlannerNode);

        // Основной цикл A*
        while (!open_set.empty()){
            // Получаем узел с наименьшей f_cost
            PlannerNode current_PlannerNode = open_set.top();
            open_set.pop();

            // Проверяем, достигли ли мы цели
            if ((current_PlannerNode.x == goal_x && current_PlannerNode.y == goal_y)){
                return reconstructPath(current_PlannerNode, closed_set);
            }

            // Рассматриваем соседей
            int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1}; // Смещения по x (8-связность)
            int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1}; // Смещение по y

            for (int i = 0; i<8; i++){
                int neighbor_x = current_PlannerNode.x + dx[i];
                int neighbor_y = current_PlannerNode.y + dy[i];

                // ПРоверяем, является ли сосед допустимым и не посещённым
                PlannerNode neighbor_PlannerNode_check(neighbor_x, neighbor_y, 0, 0);

                bool neighbor_is_visited = false;
                for (PlannerNode& node : closed_set) {
                    if(node.x == neighbor_PlannerNode_check.x && node.y == neighbor_PlannerNode_check.y){
                        neighbor_is_visited = true;
                        break;
                    }
                }

                if (isCellValid(neighbor_x, neighbor_y) && (!neighbor_is_visited) && !isCellOccupied(neighbor_x, neighbor_y)){
                    // Вычисляем g_cost для соседа
                    double tentative_g_cost = current_PlannerNode.g_cost + heuristic(current_PlannerNode.x, current_PlannerNode.y, neighbor_x, neighbor_y);

                    // Создаём новый узел для соседа
                    PlannerNode neighbor_PlannerNode(neighbor_x,
                                                     neighbor_y,
                                                     tentative_g_cost,
                                                     heuristic(neighbor_x, neighbor_y, goal_x, goal_y),
                                                     current_PlannerNode.x, current_PlannerNode.y);
                    // Добавляем соседа в open_set
                    open_set.push(neighbor_PlannerNode);
                    // Добавляем соседа в closed_set, если его там нет
                    bool isExist = false;
                    for (unsigned int n = 0; n < closed_set.size(); n++){
                        if(closed_set.at(n).x == neighbor_PlannerNode.x && closed_set.at(n).y == neighbor_PlannerNode.y){
                            isExist = true;
                            break;
                        }
                    }
                    if(!isExist){
                        closed_set.push_back(neighbor_PlannerNode);
                    }

                }
            }
        }

        // Если путь не найден, возвращаем пустой вектор
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }


    // Фильтрация пути /////////////////////////////////////////
    bool isLineOfSight(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0, y = y0;

        while (true) {
         // Если ячейка занята (препятствие)
            if (isCellOccupied(x, y)) {
                return false;
            }

            if (x == x1 && y == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
                }
            if (e2 < dx) {
                err += dx;
                y += sy;
                }
            }
        return true;

    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) {

        if (raw_path.size() <= 2) {
            return raw_path; // Нечего сглаживать, всего 2 точки
        }

        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
        smoothed_path.push_back(raw_path[0]); // Всегда добавляем первую точку

        int current_idx = 0;
        int i = 2; // Начинаем с третьей точки

        while (i < raw_path.size()) {
            // Преобразуем координаты в ячейки карты
            int x0, y0, x1, y1;
            // Точка отсчёта
            worldToMap(raw_path[current_idx].pose.position.x, raw_path[current_idx].pose.position.y, x0, y0);

            // Точка до которой смотрим
            worldToMap(raw_path[i].pose.position.x, raw_path[i].pose.position.y, x1, y1);

            if (isLineOfSight(x0, y0, x1, y1)) {
                // Препятствия нет - пробуем следующую точку
                i++;
            }
            else {
                // Препятствия есть - добавляем предыдущую точку как ключевую
                smoothed_path.push_back(raw_path[i - 1]);
                current_idx = i - 1;
                i = current_idx + 2;
            }
        }

        // Всегда добавляем последнюю точку
        smoothed_path.push_back(raw_path.back());

        RCLCPP_INFO(this->get_logger(),
        "Path smoothed: %zu points → %zu points",
        raw_path.size(), smoothed_path.size());

        return smoothed_path;
    }

    ///////////////////////////////////////////////////////////



    // Публикация пути (для визуализации RViz)
    void publishPath(std::vector<geometry_msgs::msg::PoseStamped>& path){
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        for (unsigned int i = 0; i<path.size(); i++){
            geometry_msgs::msg::PoseStamped pose = path.at(i);
            pose.header = path_msg.header;
            path_msg.poses.push_back(pose);
        }

        publisher->publish(path_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPlanner>());
    rclcpp::shutdown();
    return 0;
}







































