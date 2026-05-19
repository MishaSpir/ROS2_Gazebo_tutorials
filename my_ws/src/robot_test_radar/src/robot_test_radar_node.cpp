#include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // для иметации обратной связи с колёс
#include <cmath>
#include <std_msgs/msg/int8.hpp>  // Для публикации направления

#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// #include <tf2_ros/transform_broadcaster.h> // для созданние бродкастера трансформации
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>  
// #include <Eigen/Dense> // для матриц
// #include <pcl/point_types.h>
// #include <pcl/registration/gicp.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node
{
public:
    RobotNode()
        :Node("robot_slam_node")
        {

        this->declare_parameter("MaximumIterations", 100);
        this->declare_parameter("EuclideanFitnessEpsilon", 1.0);     
        this->declare_parameter("MaxCorrespondenceDistance", 1.0); 
        this->declare_parameter("Leaf", 1.0);
        this->declare_parameter("match_distance_pointcld", 1.0);
                 


        
        icp_max_iterations = get_parameter("MaximumIterations").as_int();
        icp_epsilon = get_parameter("EuclideanFitnessEpsilon").as_double();
        icp_max_distance = get_parameter("MaxCorrespondenceDistance").as_double();
        leaf = get_parameter("Leaf").as_double();
        match_distance = get_parameter("match_distance_pointcld").as_double();


    
        // pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1); // Публикатор карты
        // pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 1); // Публикатор позы EKF
        // // Подписчик на сканы лидара (топик /scan, очередь 10 сообщений)
        scan_sub = create_subscription<sensor_msgs::msg::Range>(
            "/ultrasonic_range", 10, std::bind(&RobotNode::scanCallback,
                      this, std::placeholders::_1));
        joint_sub = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&RobotNode::joint_callback, this, std::placeholders::_1));
        point_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar_point_cloud", 10);

        dynamic_point_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/dynamic_point_cloud", 10);    
        static_point_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/static_point_cloud", 10);    

        direction_sub = create_subscription<std_msgs::msg::Int8>(
            "/radar_direction", 10,
            std::bind(&RobotNode::direction_callback, this, std::placeholders::_1));
            

            current_direction = 1;
            scan_number = 0;
        // // Подписчик на одометрию (топик /odom, очередь 1 сообщение)
        // odom_sub =
        //     this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
        //                                                               std::bind(&RobotSlam::odomCallback, this,
        //                                                                                     std::placeholders::_1));              
        // tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this); // Широковещатель трансформации
        
        // sm.setMode(MatcherMode::Multiscan); // Установка режима накопления карты
        // // Таймер на 1 секунду — основной цикл обработки
        timer = this->create_wall_timer(100ms, std::bind(&RobotNode::timer_callback, this));
    }



    void timer_callback() {
        current_scan.header.stamp = this->get_clock()->now();  // Текущая метка времени
        range = current_scan.range;
        range = std::min(std::max(range,0.0f),5.0f);

        // RCLCPP_INFO_STREAM(this->get_logger(),"range: "<< range);
        publishPointCloud();


    
    }        


    // Нормализация угла в диапазон [-M_PI, M_PI)
    double normalizeAngle(double angle) {
        // Приводим к диапазону [0, 2π)
        angle = fmod(angle, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;

        // Преобразуем в [-π, π) если нужно
        if (angle >= M_PI) angle -= 2 * M_PI;

        return angle;
    }

    //     this->publishTF();  
    // }            
    
void scanCallback(const sensor_msgs::msg::Range &scan) {
    current_scan = scan;
    is_new_scan = true;

    processMeasurement();
        
    } 
void joint_callback(const sensor_msgs::msg::JointState & msg){
    for (size_t i = 0; i < msg.name.size(); ++i) {
            if (msg.name[i] == "radar_joint") {
                current_radar_angle = msg.position[i];
                has_angle = true;
                break;
            }
        }
    current_radar_angle = normalizeAngle(current_radar_angle);    
    // RCLCPP_INFO_STREAM(this->get_logger(),"current_radar_angle: "<< current_radar_angle);    
}

void direction_callback(const std_msgs::msg::Int8::SharedPtr msg){
    int new_direction = msg->data;
    if (current_direction != new_direction) {
        int old_direction = current_direction;
        current_direction = new_direction;

    }
    RCLCPP_INFO_STREAM(this->get_logger(),"new_direction: "<< new_direction);
    scan_number++;
    compareScansWithICP();


     if (scan_number >= 1 && !points_x.empty()) {
            scan_number = 0;
            // Сравниваем текущий и предыдущий скан через ICP
            points_x.clear();
            points_y.clear();

        }
        
}
void processMeasurement() {
        if (!is_new_scan ) return;
        if (!has_angle) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Waiting for radar angle...");
            is_new_scan = false;
            return;
        }
        
        float range_val = current_scan.range;
        
        // Проверяем валидность измерения
        if (range_val >= current_scan.min_range && 
            range_val <= current_scan.max_range) {
            
            // Преобразуем полярные координаты в декартовы
            // Точка в системе координат радара
            double x = range_val * cos(current_radar_angle);
            double y = -range_val * sin(current_radar_angle);
            
            // Добавляем точку в облако
            points_x.push_back(x);
            points_y.push_back(y);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Added point: angle=%.3f rad, range=%.3f m → (%.3f, %.3f) m",
                current_radar_angle, range_val, x, y);
        }
        
        is_new_scan = false;
    }

    void publishPointCloud() {
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        
        // Заполняем заголовок
        cloud_msg->header.stamp = this->get_clock()->now();
        cloud_msg->header.frame_id = "radar_link";  // Система координат радара
        
        // Настраиваем формат облака точек
        cloud_msg->height = 1;  // Обычное облако (не организованное)
        cloud_msg->width = points_x.size();
        cloud_msg->is_dense = true;
        cloud_msg->is_bigendian = false;
        
        // Определяем поля точки: x, y, z
        sensor_msgs::msg::PointField field_x, field_y, field_z;
        
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;
        
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
        
        cloud_msg->fields = {field_x, field_y, field_z};
        
        // Размер одной точки в байтах (3 поля * 4 байта = 12)
        cloud_msg->point_step = 12;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        
        // Заполняем данные
        cloud_msg->data.resize(cloud_msg->row_step);
        
        for (size_t i = 0; i < points_x.size(); ++i) {
            float x = static_cast<float>(points_x[i]);
            float y = static_cast<float>(points_y[i]);
            float z = 0.0f;
            
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 0], &x, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 4], &y, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 8], &z, sizeof(float));
        }
        
        point_cloud_pub->publish(std::move(cloud_msg));
    }    

private:
    // // Публикатор объединённого облака точек (карты) в топик /map_cloud
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // Подписчик на сканы радара в топик /ultrasonic_range
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_point_cloud_pub;
    sensor_msgs::msg::Range current_scan;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr direction_sub;
    bool is_new_scan = false;
    bool has_angle = true;
    int current_direction;
    int scan_number;        

    // Облако точек
    std::vector<double> points_x;
    std::vector<double> points_y;

    // Таймер для основного цикла обработки (1 Гц)
    rclcpp::TimerBase::SharedPtr timer;

    float range;
    double current_radar_angle;



    // PCL типы
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    
    // Параметры ICP
    int icp_max_iterations ;
    double icp_epsilon ;
    double icp_max_distance;
    
    double match_distance;

    double leaf;
    
    // Преобразование из точек в PCL облако
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsToPCLCloud(const std::vector<double>& points_x,
                                                          const std::vector<double>& points_y) {
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->reserve(points_x.size());
        
        for (size_t i = 0; i < points_x.size(); ++i) {
            pcl::PointXYZ point;
            point.x = points_x[i];
            point.y = points_y[i];
            point.z = 0.0;
            cloud->push_back(point);
        }
        return cloud;
    }
    
    // Функция для поиска динамических точек через ICP
    std::vector<size_t> findDynamicPoints(const std::vector<double>& prev_x,
                                           const std::vector<double>& prev_y,
                                           const std::vector<double>& curr_x,
                                           const std::vector<double>& curr_y) {
        
        if (prev_x.empty() || curr_x.empty()) {
            return {};
        }
        
        // Конвертируем в PCL облака
        auto prev_cloud = pointsToPCLCloud(prev_x, prev_y);
        auto curr_cloud = pointsToPCLCloud(curr_x, curr_y);
        
        // Фильтрация вокселями для уменьшения шума
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(leaf, leaf, leaf);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        voxel.setInputCloud(prev_cloud);
        voxel.filter(*prev_filtered);
        
        voxel.setInputCloud(curr_cloud);
        voxel.filter(*curr_filtered);
        
        // ICP для выравнивания облаков
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_filtered);
        icp.setInputTarget(curr_filtered);
        icp.setMaximumIterations(icp_max_iterations);
        icp.setEuclideanFitnessEpsilon(icp_epsilon);
        icp.setMaxCorrespondenceDistance(icp_max_distance);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);
        
        if (!icp.hasConverged()) {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge!");
            return {};
        }
        
        // Получаем трансформацию
        Eigen::Matrix4f transform = icp.getFinalTransformation();
        float fitness = icp.getFitnessScore();
        
        RCLCPP_INFO(this->get_logger(), "ICP fitness: %.4f", fitness);
        
        // Применяем трансформацию к текущему облаку
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_curr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*curr_filtered, *transformed_curr, transform);
        
        // Находим динамические точки (которые не совпадают с предыдущим облаком)
        std::vector<size_t> dynamic_indices;
        match_distance = 0.2;  // порог совпадения
        
        for (size_t i = 0; i < transformed_curr->size(); ++i) {
            const auto& curr_point = (*transformed_curr)[i];
            bool has_match = false;
            
            // Ищем ближайшую точку в предыдущем облаке
            for (const auto& prev_point : *prev_filtered) {
                double dx = curr_point.x - prev_point.x;
                double dy = curr_point.y - prev_point.y;
                double dist = sqrt(dx*dx + dy*dy);
                
                if (dist < match_distance) {
                    has_match = true;
                    break;
                }
            }
            
            if (!has_match) {
                dynamic_indices.push_back(i);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Found %zu dynamic points out of %zu total", 
                   dynamic_indices.size(), transformed_curr->size());
        
        return dynamic_indices;
    }
    
    // Обновленная версия сравнения сканов
    void compareScansWithICP() {
        if (previous_points_x.empty() || points_x.empty()) {
            previous_points_x = points_x;
            previous_points_y = points_y;
            return;
        }
        
        // Находим динамические точки
        std::vector<size_t> dynamic_idx = findDynamicPoints(
            previous_points_x, previous_points_y,
            points_x, points_y);
        
        // Визуализируем динамические точки
        dynamic_points_x.clear();
        dynamic_points_y.clear();
        
        for (size_t idx : dynamic_idx) {
            if (idx < points_x.size()) {
                dynamic_points_x.push_back(points_x[idx]);
                dynamic_points_y.push_back(points_y[idx]);
            }
        }
        
        // Сохраняем текущий скан для следующего сравнения
        previous_points_x = points_x;
        previous_points_y = points_y;
        
        // Публикуем динамические точки (красным цветом)
        publishStaticPoints();     
        publishDynamicPoints();
    }
    
    // Публикация динамических точек
    void publishDynamicPoints() {
        if (dynamic_points_x.empty()) return;
        
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.stamp = this->get_clock()->now();
        cloud_msg->header.frame_id = "radar_link";
        cloud_msg->height = 1;
        cloud_msg->width = dynamic_points_x.size();
        cloud_msg->is_dense = true;
        
        // Настраиваем поля
        sensor_msgs::msg::PointField field_x, field_y, field_z;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;
        
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
        
        cloud_msg->fields = {field_x, field_y, field_z};
        cloud_msg->point_step = 12;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize(cloud_msg->row_step);
        
        for (size_t i = 0; i < dynamic_points_x.size(); ++i) {
            float x = static_cast<float>(dynamic_points_x[i]);
            float y = static_cast<float>(dynamic_points_y[i]);
            float z = 0.0f;
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 0], &x, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 4], &y, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 8], &z, sizeof(float));
        }
        
        dynamic_point_cloud_pub->publish(std::move(cloud_msg));
    }
    
        // Публикация статических точек (зеленым цветом)
    void publishStaticPoints() {
        if (previous_points_x.empty()) return;

        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.stamp = this->get_clock()->now();
        cloud_msg->header.frame_id = "radar_link";
        cloud_msg->height = 1;
        cloud_msg->width = previous_points_x.size();
        cloud_msg->is_dense = true;

        // Настраиваем поля (x, y, z)
        sensor_msgs::msg::PointField field_x, field_y, field_z;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;

        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;

        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;

        cloud_msg->fields = {field_x, field_y, field_z};
        cloud_msg->point_step = 12;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize(cloud_msg->row_step);

        for (size_t i = 0; i < previous_points_x.size(); ++i) {
            float x = static_cast<float>(previous_points_x[i]);
            float y = static_cast<float>(previous_points_y[i]);
            float z = 0.0f;
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 0], &x, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 4], &y, sizeof(float));
            memcpy(&cloud_msg->data[i * cloud_msg->point_step + 8], &z, sizeof(float));
        }

        static_point_cloud_pub->publish(std::move(cloud_msg));
    }

    // Переменные для хранения
    std::vector<double> previous_points_x;
    std::vector<double> previous_points_y;
    std::vector<double> dynamic_points_x;
    std::vector<double> dynamic_points_y;

    // // Броадкастер для публикации TF трансформаций (map -> odom)
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    //  // Публикатор позы робота (от EKF) в топик /ekf_pose
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

   
    // // Текущий лазерный скан (последнее полученное сообщение)
    // sensor_msgs::msg::LaserScan current_scan;

    // // Параметры шумов (launch)
    // std::vector<double> motion_noise;  // Шум модели движения [dx, dy, dtheta]
    // std::vector<double> measurement_noise;  // Шум измерений ICP [x, y, theta]
    // // Объект класса сопоставления сканов (ICP + фильтры)
    // ScanMatcher sm;
    // // Флаг наличия нового скана для обработки
    // bool is_new_scan;


    // //=============ОДОМЕТРИЯ
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    // geometry_msgs::msg::Pose odom_previous_pose;  // Предыдущая поза из одометрии
    // geometry_msgs::msg::Pose odom_current_pose;   // Текущая поза из одометрии
    // bool odom_has_previous_pose;  // Флаг инициализации (была ли получена первая поза)

    // //=============Переменные фильтр Калмана (EKF)
    // Eigen::Vector3d x_hat;  // Оценка состояния [x, y, theta] — поза робота
    // Eigen::Matrix3d P;  // Матрица ковариации ошибок оценки состояния
    // Eigen::Matrix3d P_predicted;  // Предсказанная ковариация (на шаге прогноза)
    // Eigen::Matrix3d Q;  // Матрица ковариации шума модели движения (берем из одометрии)
    // Eigen::Matrix3d R;  // Матрица ковариации шума измерений (берем из алгоритма icp)

};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;


}



