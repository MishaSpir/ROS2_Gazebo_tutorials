#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h> // для созданние бродкастера трансформации
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>  
#include <Eigen/Dense> // для матриц
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



enum MatcherMode {
    Pairwise,       // Сопоставление только двух последовательных сканов
    Multiscan       // Накопление всех сканов в глобальную карту
};


// Класс ScanMatcher — сопоставление лазерных сканов с помощью ICP
class ScanMatcher
{
public:
    ScanMatcher(int MaxIters, float EuclidFitEps, float MaxCorrespondDist, float leaf_in, float meank_in, float StddevMulThresh_in):
        global_transformation(Eigen::Matrix4f::Identity()),  // Начальная трансформация (положение работа в пр-ве по данным сопаставления). Вначале она единичная 4х4
        fitnessScore(0.01)  // Начальное значение функции качества
    {
        
        leaf = leaf_in;  // Размер вокселя для фильтра downsampling (м)
        MaximumIterations = MaxIters;  // Максимальное число итераций ICP
        EuclideanFitnessEpsilon = EuclidFitEps;  // Порог сходимости ICP по ошибке
        MaxCorrespondenceDistance = MaxCorrespondDist;  // Макс. расстояние для соответствия точек (м)
        MeanK = meank_in;  // Количество соседей для статистического фильтра выбросов
        StddevMulThresh = StddevMulThresh_in;  // Порог стандартного отклонения для удаления выбросов
    
    }
    // режим сопоставления
    void setMode(MatcherMode m){
        mode = m;
    } 
    // Алгоритм сопоставления сканов, возвращает матр 4х4 трансвормаицию положения между двумя сканами
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
        icp.setMaximumIterations(MaximumIterations);   // Максимальное количество итераций
        icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon); // Критерий сходимости по ошибке
        icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance); // Макс. дистанция соответствия точек
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Запуск ICP с начальным приближением от одометрии (odom_tf)
        icp.align(*output, odom_tf);// Начинаем поиск с позиции odom_tf
        fitnessScore = icp.getFitnessScore();  // Получение метрики качества совпадения
        std::cout<<"Fitness score: "<<fitnessScore<<std::endl;  // Вывод в терминал

        // Накопление глобальной трансформации: умножаем на обратную трансформацию ICP
        global_transformation = global_transformation * icp.getFinalTransformation().inverse();
        // std::cout<<"Global transformation"<<std::endl<<global_transformation<<std::endl;


        // Обновление предыдущего облака в зависимости от режима
        if (mode == MatcherMode::Pairwise) {
            // Режим Pairwise: просто заменяем предыдущее облако на текущее
            prev_cloud = cloud;
        }
        else if(mode == MatcherMode::Multiscan){
            // Режим Multiscan: трансформируем накопленное облако по найденной трансформации
            pcl::transformPointCloud(*prev_cloud, *prev_cloud, icp.getFinalTransformation());
            *prev_cloud += *cloud;  // Добавляем текущий скан к накопленному облаку
            
            Eigen::Matrix4f transform = icp.getFinalTransformation();
            float translation = sqrt(transform(0,3)*transform(0,3) + transform(1,3)*transform(1,3));
            float rotation = atan2(transform(1,0), transform(0,0));
            
            if (translation > 0.5 || fabs(rotation) > 0.5) {  // >50 см или >30 градусов
                RCLCPP_WARN(rclcpp::get_logger("ScanMatcher"), 
                "Suspicious large transform: trans=%.2f, rot=%.2f, skipping",
                translation, rotation);
                return global_transformation;  // Пропускаем этот скан
            }            
            // Проверка, не слишком ли большая трансформация
            

            // ФИЛЬТР1 - Фильтрация вокселями (уменьшение плотности облака для производительности)
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(prev_cloud);
            vox.setLeafSize(leaf, leaf, leaf);  // Размер ячейки воксельной сетки (м)
            vox.filter (*prev_cloud);

            // ФИЛЬТР2 - Удаление статистических выбросов (шумовые точки)
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(prev_cloud);
            sor.setMeanK(MeanK);  // Количество соседей для анализа
            sor.setStddevMulThresh(StddevMulThresh);  // Порог в стандартных отклонениях
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

    // Параметры ICP и фильтров
    int MaximumIterations;   // Максимальное количество итераций ICP
    float EuclideanFitnessEpsilon; // Порог сходимости ICP по ошибке
    float MaxCorrespondenceDistance; // Максимальное расстояние для соответствия точек
    float leaf;  // Размер вокселя для фильтра downsampling
    float MeanK;  // Количество соседей для статистического фильтра
    float StddevMulThresh;  // Порог стандартного отклонения для удаления выбросов

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
        motion_noise(declare_parameter("motion_noise", std::vector<double>{0.01, 0.01, 0.001})),
        // Чтение параметров шума измерений из launch-файла (по умолчанию [0.05, 0.05, 0.01])
        measurement_noise(declare_parameter("measurement_noise", std::vector<double>{0.05, 0.05, 0.01})),
        // Инициализация ScanMatcher с параметрами ICP и фильтров
        sm(declare_parameter("MaximumIterations", 100),
            declare_parameter("EuclideanFitnessEpsilon", 1e-6),
            declare_parameter("MaxCorrespondenceDistance", 0.3),
            declare_parameter("Leaf", 0.1f),
            declare_parameter("MeanK", 30),
            declare_parameter("StddevMulThresh", 1.5)),
        is_new_scan(false),
        odom_has_previous_pose(false)
    {
        this->declare_parameter("timer_period_ms", 50);
        int period_ms = this->get_parameter("timer_period_ms").as_int();
        auto period = std::chrono::milliseconds(period_ms);



        auto motion_noise_params = motion_noise;
        auto measurement_noise_params = measurement_noise;
        // Инициализация матриц ковариации шума
        Q = Eigen::Matrix3d::Zero();
        Q(0, 0) = motion_noise_params[0]; // dx — шум движения по оси X
        Q(1, 1) = motion_noise_params[1]; // dy — шум движения по оси Y
        Q(2, 2) = motion_noise_params[2]; // dtheta — шум поворота

        R = Eigen::Matrix3d::Zero();
        R(0, 0) = measurement_noise_params[0]; // x_icp — шум измерения X от ICP
        R(1, 1) = measurement_noise_params[1]; // y_icp — шум измерения Y от ICP
        R(2, 2) = measurement_noise_params[2]; // theta_icp — шум измерения угла от ICP

        // Инициализация состояния и ковариации
        x_hat = Eigen::Vector3d::Zero(); // [x=0, y=0, theta=0] — начальное состояние робота
        P = Eigen::Matrix3d::Identity() * 0.1; // Начальная ковариация (неуверенность в состоянии)


        //=================================================================================================
        pointCloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1); // Публикатор карты
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 1); // Публикатор позы EKF
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
        timer = this->create_wall_timer(period, std::bind(&RobotSlam::timer_callback, this));
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

            // Получение данных одометрии: пройденное расстояние между кадрами
            double delta_d = sqrt(pow(odom_transformation(0, 3), 2) + pow(odom_transformation(1, 3), 2));
            // Извлекаем матрицу вращения 3х3 из трансформации
            Eigen::Matrix3f rotation_matrix = odom_transformation.block<3,3>(0,0);
            // Предполагаем ZYX (yaw, pitch, roll) порядок вращения
            // Формула выведена из того, как строится матрица вращения при ZYX
            double delta_theta = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
            // Вектор управления u_t = [delta_d, delta_theta]
            Eigen::Vector2d u_t;
            u_t << delta_d, delta_theta;
            this->predict(u_t);  // ШАГ 1 EKF: Предсказание состояния по модели движения

            // ШАГ 2 EKF: Получение измерения от ICP
            Eigen::Matrix4f icp_transformation = sm.addAndMatchScan(current_scan,odom_transformation);
            
            // Извлекаем матрицу вращения из ICP трансформации
            rotation_matrix = icp_transformation.block<3,3>(0,0);

            // Предполагаем ZYX (yaw, pitch, roll) порядок вращения.
            // Вычисляем угол поворота (yaw) из матрицы вращения
            double theta_icp = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

            // Получение измерения позы от ICP: z_t = [x_icp, y_icp, theta_icp]
            Eigen::Vector3d z_t;
            z_t << icp_transformation(0,3), icp_transformation(1,3), theta_icp;
            correct(z_t);  // ШАГ 3 EKF: Коррекция состояния по измерению ICP

            odom_previous_pose = odom_current_pose;
            is_new_scan = false;

            // Публикация сопоставленного облака для визуализации 
            sensor_msgs::msg::PointCloud2 pointCloud;
            pointCloud = *sm.getMergedPointCloud();
            pointCloud.header.stamp = now();  // Установка текущей метки времени
            pointCloud.header.frame_id = current_scan.header.frame_id;  // Установка фрейма
            pointCloud_pub->publish(pointCloud);


        }

        this->publishTF();  
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


    // ШАГ 1 EKF: Предсказание Новая оценка позиции  по модели движения
    // вход - оценка вектора перемещения
    void predict(const Eigen::Vector2d& u_t){
        // 1. Новая оценка позиции 
        double delta_d = u_t(0);  // Пройденное расстояние
        double delta_theta = u_t(1);  // Угол поворота
        double theta_prev = x_hat(2);  // Предыдущий угол (yaw)

        // Модель движения: предсказание новой позы
        Eigen::Vector3d x_hat_predicted;
        x_hat_predicted(0) = x_hat(0) + delta_d * cos(theta_prev + delta_theta / 2.0);  // x
        x_hat_predicted(1) = x_hat(1) + delta_d * sin(theta_prev + delta_theta / 2.0);  // y
        x_hat_predicted(2) = x_hat(2) + delta_theta;

        // 2. Вычисление матрицы Якоби модели движения F_x = ∂f(x_{t-1}, u_t)/∂x
        // Матрица Якоби нужна для линеаризации нелинейной модели движения
        Eigen::Matrix3d F_x = Eigen::Matrix3d::Identity();
        F_x(0, 2) = -delta_d * sin(theta_prev + delta_theta / 2.0);  // ∂x/∂theta
        F_x(1, 2) = delta_d * cos(theta_prev + delta_theta / 2.0);   // ∂y/∂theta

        // 3. Предсказание ковариации: P_{t|t-1} = F_x * P_{t-1|t-1} * F_x^T + Q
        // Q — ковариация шума модели движения
        P_predicted = F_x * P * F_x.transpose() + Q;

        // Обновление предсказанного состояния
        x_hat = x_hat_predicted;  // Замена состояния на предсказанное
        P = P_predicted; // Для следующей итерации предсказания, но для коррекции используем P_predict
    }

     // ШАГ 2 EKF: Коррекция состояния по измерению ICP
     // вход - измерительные данные он состоит из x y и поворот от icp
    void correct(const Eigen::Vector3d& z_t){
        // 1. Вычисление матрицы Якоби модели измерений H_x
        // В данном случае H_x = I, т.к. измеряем состояние напрямую (x, y, theta)
        Eigen::Matrix3d H_x = Eigen::Matrix3d::Identity();

        // 2. Вычисление инновации (ошибка измерения): y_t = z_t - h(x_hat_{t|t-1})
        // h(x_hat) = x_hat, т.к. измерение напрямую соответствует состоянию
        Eigen::Vector3d y_t = z_t - x_hat; // Вектор невязки между измерением и предсказанием

        // 3. Вычисление ковариации инновации: S_t = H_x * P_{t|t-1} * H_x^T + R
        // Поскольку H_x = I, формула упрощается до: S_t = P_predicted + R
        // Eigen::Matrix3d S_t = P_predicted + R; // Используем предсказанную ковариацию
        Eigen::Matrix3d S_t = P + R;  // P уже содержит P_predicted
        // 4. Вычисление усиления Калмана: K_t = P_{t|t-1} * H_x^T * S_t^{-1}
        // Поскольку H_x = I, формула упрощается до: K_t = P_predicted * S_t^{-1}
        Eigen::Matrix3d K_t = P_predicted * H_x.transpose() * S_t.inverse();
        // Eigen::Matrix3d K_t = P * H_x.transpose() * S_t.inverse();  

        // 5. Обновление состояния: x_hat_{t|t} = x_hat_{t|t-1} + K_t * y_t
        x_hat = x_hat + K_t * y_t;

        // Выводим предсказанное состояние в терминал (отладочная информация)
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "EKF estimated \n"<<x_hat);

        // 6. Обновление ковариации: P_{t|t} = (I - K_t * H_x) * P_{t|t-1}
        // Поскольку H_x = I, формула упрощается до: P = (I - K_t) * P_predicted
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P = (I - K_t * H_x) * P_predicted;
        // P = (I - K_t * H_x) * P;
    }


    void publishTF(){
        // ========================================================================
        // Формирование и публикация сообщения с позой робота
        // ========================================================================
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();  // Текущая метка времени
        pose_msg.header.frame_id = "map";  // Фрейм карты
        pose_msg.pose.position.x = x_hat(0);  // X координата от EKF
        pose_msg.pose.position.y = x_hat(1);  // Y координата от EKF
        pose_msg.pose.position.z = 0.0;  // 2D SLAM, z=0 (робот движется в плоскости)

        // Конвертация угла yaw в кватернион (для ROS ориентация задаётся кватернионом)
        tf2::Quaternion q;
        q.setRPY(0, 0, x_hat(2));  // Roll=0, Pitch=0, Yaw=x_hat(2)
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_publisher->publish(pose_msg);


         // ========================================================================
        // Вычисление трансформации map -> odom для TF дерева
        // ========================================================================
        
        // 1. Получаем позу одометрии в виде трансформации odom -> base_link
        tf2::Transform odom_to_base;
        odom_to_base = tf2::Transform(
            tf2::Quaternion(
                odom_current_pose.orientation.x,
                odom_current_pose.orientation.y,
                odom_current_pose.orientation.z,
                odom_current_pose.orientation.w),
            tf2::Vector3(
                odom_current_pose.position.x,
                odom_current_pose.position.y,
                odom_current_pose.position.z));

        // 2. Получаем позу от сопоставления сканов (уже трансформация) map -> base_link
        tf2::Transform map_to_base;
        map_to_base = tf2::Transform(
            tf2::Quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w),
            tf2::Vector3(
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z));

        // 3. Вычисляем map -> odom: map_to_odom = map_to_base * (odom_to_base)^-1
        // Это нужно, чтобы связать глобальную карту (map) с локальной одометрией (odom)
        tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

        // Формирование сообщения трансформации
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";  // Родительский фрейм
        transform_stamped.child_frame_id = "odom";  // Дочерний фрейм

        transform_stamped.transform = tf2::toMsg(map_to_odom);  // Конвертация в ROS сообщение

        tf_broadcaster->sendTransform(transform_stamped);  // Публикация TF
    }



private:
    // Публикатор объединённого облака точек (карты) в топик /map_cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // Подписчик на сканы лидара в топик /scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    // Броадкастер для публикации TF трансформаций (map -> odom)
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
     // Публикатор позы робота (от EKF) в топик /ekf_pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

    // Таймер для основного цикла обработки (1 Гц)
    rclcpp::TimerBase::SharedPtr timer;
   
    // Текущий лазерный скан (последнее полученное сообщение)
    sensor_msgs::msg::LaserScan current_scan;

    // Параметры шумов (launch)
    std::vector<double> motion_noise;  // Шум модели движения [dx, dy, dtheta]
    std::vector<double> measurement_noise;  // Шум измерений ICP [x, y, theta]
    // Объект класса сопоставления сканов (ICP + фильтры)
    ScanMatcher sm;
    // Флаг наличия нового скана для обработки
    bool is_new_scan;


    //=============ОДОМЕТРИЯ
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    geometry_msgs::msg::Pose odom_previous_pose;  // Предыдущая поза из одометрии
    geometry_msgs::msg::Pose odom_current_pose;   // Текущая поза из одометрии
    bool odom_has_previous_pose;  // Флаг инициализации (была ли получена первая поза)

    //=============Переменные фильтр Калмана (EKF)
    Eigen::Vector3d x_hat;  // Оценка состояния [x, y, theta] — поза робота
    Eigen::Matrix3d P;  // Матрица ковариации ошибок оценки состояния
    Eigen::Matrix3d P_predicted;  // Предсказанная ковариация (на шаге прогноза)
    Eigen::Matrix3d Q;  // Матрица ковариации шума модели движения (берем из одометрии)
    Eigen::Matrix3d R;  // Матрица ковариации шума измерений (берем из алгоритма icp)

};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotSlam>());
    rclcpp::shutdown();
    return 0;


}
