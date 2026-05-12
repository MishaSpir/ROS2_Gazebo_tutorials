#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h> // для созданние бродкастера трансформации
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>  
#include <Eigen/Dense> // для матриц
// Типы точек PCL (Point Cloud Library) — например, PointXYZRGB
#include <pcl/point_types.h>
// Generalized ICP — алгоритм сопоставления облаков точек
#include <pcl/registration/gicp.h>
// Конвертация между форматами PCL и ROS сообщений
#include <pcl_conversions/pcl_conversions.h>
// Фильтр вокселизации для уменьшения плотности облака (downsampling)
#include <pcl/filters/voxel_grid.h>
// Фильтр удаления выбросов на основе статистики соседей
#include <pcl/filters/statistical_outlier_removal.h>



enum MatcherMode {
    Pairwise,       // Сопоставление только двух последовательных сканов
    Multiscan       // Накопление всех сканов в глобальную карту
};


// Класс ScanMatcher — сопоставление лазерных сканов с помощью ICP
class ScanMatcher
{
public:
    ScanMatcher():
        global_transformation(Eigen::Matrix4f::Identity()),  // Начальная трансформация (положение работа в пр-ве по данным сопаставления). Вначале она единичная 4х4
        fitnessScore(0.01)  // Начальное значение функции качества
    {

    }
    // режим сопоставления
    void setMode(MatcherMode m){
        mode = m;
    } 
    // Алгоритм сопоставления сканов, возвращает матр 4х4 трансвормаицию положения
    Eigen::Matrix4f addAndMatchScan(const sensor_msgs::msg::LaserScan &scan,
                                    const Eigen::Matrix4f odom_tf = Eigen::Matrix4f::Identity())
    {
        // Преобразуем поступивший скан в облако точек PCL
        auto cloud = laserScanToPointCloud(scan);
        // Если это первый скан — сохраняем его и возвращаем единичную трансформацию
        if (!prev_cloud){
            prev_cloud = cloud;
            return Eigen::Matrix4f::Identity();
        }

        // ВЫполнение ICP
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(prev_cloud);  // Источник — предыдущее (накопленное) облако
        icp.setInputTarget(cloud);  // Цель — текущее облако
        icp.setMaximumIterations(100);   // Максимальное количество итераций
        icp.setEuclideanFitnessEpsilon(1e-6); // Критерий сходимости по ошибке
        icp.setMaxCorrespondenceDistance(0.5); // Макс. дистанция соответствия точек
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Запуск ICP с начальным приближением от одометрии (odom_tf)
        icp.align(*output, odom_tf);
        fitnessScore = icp.getFitnessScore();  // Получение метрики качества совпадения
        std::cout<<"Fitness score: "<<fitnessScore<<std::endl;  // Вывод в терминал

        // Накопление глобальной трансформации: умножаем на обратную трансформацию ICP
        global_transformation = global_transformation * icp.getFinalTransformation().inverse();
        std::cout<<"Global transformation"<<std::endl<<global_transformation<<std::endl;


        // Обновление предыдущего облака в зависимости от режима
        if (mode == MatcherMode::Pairwise) {
            // Режим Pairwise: просто заменяем предыдущее облако на текущее
            prev_cloud = cloud;
        }
        else if(mode == MatcherMode::Multiscan){
            // Режим Multiscan: трансформируем накопленное облако по найденной трансформации
            pcl::transformPointCloud(*prev_cloud, *prev_cloud, icp.getFinalTransformation());
            *prev_cloud += *cloud;  // Добавляем текущий скан к накопленному облаку

            // ФИЛЬТР1 - Фильтрация вокселями (уменьшение плотности облака для производительности)
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(prev_cloud);
            float leaf = 0.1f; // 4 см
            vox.setLeafSize(leaf, leaf, leaf);  // Размер ячейки воксельной сетки (м)
            vox.filter (*prev_cloud);

            // ФИЛЬТР2 - Удаление статистических выбросов (шумовые точки)
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(prev_cloud);
            sor.setMeanK(30);  // Количество соседей для анализа
            sor.setStddevMulThresh(1.5);  // Порог в стандартных отклонениях
            sor.filter(*prev_cloud);

        }


        return global_transformation;  // Возврат накопленной трансформации


    }
    // Накполенное результирующее облако точек рез-те сопоставления
    sensor_msgs::msg::PointCloud2::SharedPtr getMergedPointCloud(){
        std_msgs::msg::Header header;
        header.frame_id = prev_cloud->header.frame_id;  // Копирование имени фрейма
        return this->converToPointCloud2(prev_cloud, header);
    }
    // Оценка сопоставления
    float getFitnessScore(){
        return fitnessScore;
    }



private:
    // Режим сопоставления (Pairwise или Multiscan)
    MatcherMode mode;
    // Предыдущее (накопленное) облако точек
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud;
    // Глобальная трансформация (накопленная за всё время работы)
    Eigen::Matrix4f global_transformation;
    // Текущее значение функции качества ICP
    float fitnessScore;

    // Преобразование LaserScan в PCL (из полярных в декартовы координаты)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserScanToPointCloud(const sensor_msgs::msg::LaserScan &scan){
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cloud->header.frame_id = scan.header.frame_id;  // Копирование имени фрейма из скана

        // Генерируем случайный цвет для текущего облака (для визуализации в RViz)
        uint8_t r = static_cast<uint8_t>(rand() % 256);
        uint8_t g = static_cast<uint8_t>(rand() % 256);
        uint8_t b = static_cast<uint8_t>(rand() % 256);

        // Проход по всем лучам скана
        for (size_t i=0; i < scan.ranges.size(); i++){
            if (!std::isfinite(scan.ranges[i])) continue;  // Пропуск невалидных измерений (inf, NaN)
            float angle = scan.angle_min + i * scan.angle_increment;  // Вычисление угла луча
            pcl::PointXYZRGB point;
            point.x = scan.ranges[i] * cos(angle);  // Преобразование полярных координат в декартовы X
            point.y = scan.ranges[i] * sin(angle);  // Преобразование полярных координат в декартовы Y
            point.z = 0.0;  
            point.r = r;  // Присваивание цвета (красный)
            point.g = g;  // Присваивание цвета (зелёный)
            point.b = b;  // Присваивание цвета (синий)
            cloud->points.push_back(point);
        }
        // У нас одномерное облако точек
        cloud->width = cloud->points.size();  // Установка ширины облака (число точек)
        cloud->height = 1;  // Неорганизованное облако (1 ряд)
        cloud->is_dense = false;  // Могут быть невалидные точки

        return cloud;
    }

    // Конвертация облака PCL в сообщение ROS PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr converToPointCloud2(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std_msgs::msg::Header& header){
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_msg);  // Конвертация PCL -> ROS
        cloud_msg->header = header;  // Копирование заголовка
        return cloud_msg;
    }
};



using namespace std::chrono_literals;

class RobotSlam : public rclcpp::Node
{
public:
    RobotSlam()
        :Node("robot_slam_node"),
        is_new_scan(false),
        odom_has_previous_pose(false)
    {
        pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1); // Публикатор карты
        // Подписчик на сканы лидара (топик /scan, очередь 10 сообщений)
        scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RobotSlam::scanCallback,
                      this, std::placeholders::_1));
        // Подписчик на одометрию (топик /odom, очередь 1 сообщение)
        odom_sub =
            this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                      std::bind(&RobotSlam::odomCallback, this,
                                                                                            std::placeholders::_1));              
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this); // Широковещатель трансформации
        
        sm.setMode(MatcherMode::Multiscan); // Установка режима накопления карты
        // Таймер на 1 секунду — основной цикл обработки
        timer = this->create_wall_timer(400ms, std::bind(&RobotSlam::timer_callback, this));
    }

    void scanCallback(const sensor_msgs::msg::LaserScan &scan) {
        current_scan = scan;
        is_new_scan = true;        

    }

    void odomCallback(const nav_msgs::msg::Odometry &odom){
        odom_current_pose = odom.pose.pose;  // Сохранение текущей позы из одометрии
    }

    // Основной цикл обработки (вызывается таймером каждые 1000 мс)
    void timer_callback() {
        if(is_new_scan){ // Проверка флага: присутствует ли новый скан

            
            // Инициализация: запоминание первой позы одометрии
            if (!odom_has_previous_pose) {
                odom_previous_pose = odom_current_pose;
                odom_has_previous_pose = true;
                return;  // Выход до следующего скана
            }

            // Если инициализация проведена, то продолжаем
            Eigen::Matrix4f odom_transformation = Eigen::Matrix4f::Identity();
            // Если есть предыдущая поза, вычисляем разницу

            // Вычисляем трансформацию между текущей и предыдущей позой одометрии
            odom_transformation = computeTransformation(odom_previous_pose, odom_current_pose);


            // Получение данных от ICP
            Eigen::Matrix4f icp_transformation = sm.addAndMatchScan(current_scan,odom_transformation);
            odom_previous_pose = odom_current_pose;

            is_new_scan = false;

            // Публикация сопоставленного облака для визуализации 
            sensor_msgs::msg::PointCloud2 pointCloud;
            pointCloud = *sm.getMergedPointCloud();
            pointCloud.header.stamp = now();  // Установка текущей метки времени
            pointCloud.header.frame_id = current_scan.header.frame_id;  // Установка фрейма
            pointCloud_pub->publish(pointCloud);


        }
    }            
    
    // Вычисление относительной трансформации между двумя позами
    // Возвращает матрицу 4x4 (трансформация в системе координат prev_pose)
    Eigen::Matrix4f computeTransformation(
        const geometry_msgs::msg::Pose& prev_pose,
        const geometry_msgs::msg::Pose& curr_pose) {

        // Извлекаем позиции (x, y, z) из поз
        Eigen::Vector3f prev_position(
            prev_pose.position.x,
            prev_pose.position.y,
            prev_pose.position.z);

        Eigen::Vector3f curr_position(
            curr_pose.position.x,
            curr_pose.position.y,
            curr_pose.position.z
            );

        // Извлекаем ориентации (кватернионы) из поз
        Eigen::Quaternionf prev_quat(
            prev_pose.orientation.w,
            prev_pose.orientation.x,
            prev_pose.orientation.y,
            prev_pose.orientation.z
            );

        Eigen::Quaternionf curr_quat(
            curr_pose.orientation.w,
            curr_pose.orientation.x,
            curr_pose.orientation.y,
            curr_pose.orientation.z
            );

        // Вычисляем разницу в ориентации
        // Для этого умножаем текущий кватернион на обратный предыдущий
        Eigen::Quaternionf delta_quat = curr_quat * prev_quat.inverse();

        // Вычисляем разницу в позиции в системе координат предыдущей позы
        // Сначала переводим текущую позицию в систему координат предыдущей позы
        Eigen::Vector3f delta_position = prev_quat.inverse() * (curr_position - prev_position);

        // Создаём матрицу трансформации 4x4 (единичная матрица)
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        // Заполняем матрицу вращения (верхний левый угол 3x3)
        transformation.block<3,3>(0,0) = delta_quat.toRotationMatrix();

        // Заполняем вектор переноса (правый столбец 3x1)
        transformation.block<3,1>(0,3) = delta_position;

        return transformation;

    }

    void publishTF(){
        // geometry_msgs::msg::TransformStamped transform_stamped;

        // transform_stamped.header.stamp = now();
        // transform_stamped.header.frame_id = "odom";
        // transform_stamped.child_frame_id = "base_link";


        // // параметры перемещения
        // transform_stamped.transform.translation.x = x;
        // transform_stamped.transform.translation.y = y;
        // transform_stamped.transform.translation.z = 0.0;

        // tf2::Quaternion q;
        // q.setRPY(0,0, theta);
        // transform_stamped.transform.rotation.x = q.x();
        // transform_stamped.transform.rotation.y = q.y();
        // transform_stamped.transform.rotation.z = q.z();
        // transform_stamped.transform.rotation.w = q.w();

        // tf_broadcaster-> sendTransform(transform_stamped);
    }



private:
    // Публикатор объединённого облака точек (карты) в топик /map_cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // Подписчик на сканы лидара в топик /scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    // Броадкастер для публикации TF трансформаций (map -> odom)
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Таймер для основного цикла обработки (1 Гц)
    rclcpp::TimerBase::SharedPtr timer;
    // Объект класса сопоставления сканов (ICP + фильтры)
    ScanMatcher sm;
    // Текущий лазерный скан (последнее полученное сообщение)
    sensor_msgs::msg::LaserScan current_scan;
    // Флаг наличия нового скана для обработки
    bool is_new_scan;


    //=============ОДОМЕТРИЯ
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    geometry_msgs::msg::Pose odom_previous_pose;  // Предыдущая поза из одометрии
    geometry_msgs::msg::Pose odom_current_pose;   // Текущая поза из одометрии
    bool odom_has_previous_pose;  // Флаг инициализации (была ли получена первая поза)

};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotSlam>());
    rclcpp::shutdown();
    return 0;


}
