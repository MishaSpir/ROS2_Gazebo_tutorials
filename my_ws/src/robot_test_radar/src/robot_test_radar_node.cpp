#include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

class RobotSlam : public rclcpp::Node
{
public:
    RobotSlam()
        :Node("robot_slam_node")
        {
        // pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1); // Публикатор карты
        // pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 1); // Публикатор позы EKF
        // // Подписчик на сканы лидара (топик /scan, очередь 10 сообщений)
        scan_sub = create_subscription<sensor_msgs::msg::Range>(
            "/ultrasonic_range", 10, std::bind(&RobotSlam::scanCallback,
                      this, std::placeholders::_1));
        // // Подписчик на одометрию (топик /odom, очередь 1 сообщение)
        // odom_sub =
        //     this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
        //                                                               std::bind(&RobotSlam::odomCallback, this,
        //                                                                                     std::placeholders::_1));              
        // tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this); // Широковещатель трансформации
        
        // sm.setMode(MatcherMode::Multiscan); // Установка режима накопления карты
        // // Таймер на 1 секунду — основной цикл обработки
        timer = this->create_wall_timer(1000ms, std::bind(&RobotSlam::timer_callback, this));
    }



    void timer_callback() {
        current_scan.header.stamp = this->get_clock()->now();  // Текущая метка времени
        range = current_scan.range;
        range = std::min(std::max(range,0.0f),5.0f);

        RCLCPP_INFO_STREAM(this->get_logger(),"range: \n"<< range);
    
    }        

    //     this->publishTF();  
    // }            
    
void scanCallback(const sensor_msgs::msg::Range &scan) {
        current_scan = scan;
        is_new_scan = true;
        
    }   


private:
    // // Публикатор объединённого облака точек (карты) в топик /map_cloud
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // Подписчик на сканы радара в топик /ultrasonic_range
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr scan_sub;
    sensor_msgs::msg::Range current_scan;
    bool is_new_scan = true;        


    // Таймер для основного цикла обработки (1 Гц)
    rclcpp::TimerBase::SharedPtr timer;

    float range;

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
    rclcpp::spin(std::make_shared<RobotSlam>());
    rclcpp::shutdown();
    return 0;


}
