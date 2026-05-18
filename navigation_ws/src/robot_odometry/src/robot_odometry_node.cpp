#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h> // для созданние публикатора трансформации
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>  
#include <Eigen/Dense> // для матриц



using namespace std::chrono_literals;

class RobotOdometry : public rclcpp::Node
{


    public:
             RobotOdometry()
    :Node("robot_odometry_node"),
    wheel_radius(declare_parameter("wheel_radius",0.05)),
    wheel_width(declare_parameter("wheel_width",0.02)),
    wheel_base(declare_parameter("wheel_base",0.26)),
    // кол-во импульсов на один оборотт энкодера
    tiks_per_revolution(declare_parameter("tiks_per_revolution",400.0)),
    // предыдущие заченения энкодеров
    encoder_right_prev(0.0),
    encoder_left_prev(0.0),
    x(0.0),
    y(0.0),
    theta(0.0),
    last_time(now())
    {
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom",10);
        joint_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states",10,std::bind(&RobotOdometry::jointCallback,
                                                                                                    this,
                                                                                                    std::placeholders::_1));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this); // Широковещатель трансформации

        // Инициализация матрицы ковариации шума измерений R
        // 2 датчика => матрица 2х2
        R = Eigen::Matrix2d::Zero();
        // угловое разрешение = 2pi / кол-во импульсов на одиин оборот
        double encoder_angular_resolution = (2.0*M_PI) / tiks_per_revolution;
        // линейное разрешение
        double encoder_linear_resolution = wheel_radius * encoder_angular_resolution;
        // Значение дисперсии для леовго и правого колеса
        double encoder_linear_variance = std::pow(encoder_linear_resolution / 2.0,2);
        R(0,0) = encoder_linear_variance; // Левое колес
        R(1,1) = encoder_linear_variance; // Правое колесо


        // Иициализация матрицы ковариации шума процесса Q
        // Мы оцениваем 3 параметра => матрица 3х3
        Q = Eigen::Matrix3d::Zero();
        // X Y tetha
        Q(0,0) = 5.78e-7;
        Q(1,1) = 5.78e-7;
        Q(2,2) = 2.2e-7;

        // Инициализация матрицы ковариации состояния P
        P = Eigen::Matrix3d::Zero();

    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp; // считываем момент времени , когда получили измерение
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        double encoder_left_current = 0.0;
        double encoder_right_current = 0.0;

        // поиск значений с энкодеров = проверка есть ли в массивах JointState имена джоинтов
        for(size_t i = 0; i < msg->name.size(); ++i ){
            if(msg->name[i] == "left_wheel_joint"){
                encoder_left_current = msg->position[i];
            }else if (msg->name[i] == "right_wheel_joint"){
                encoder_right_current = msg->position[i];
            }

        }

        // Находим разницу
        double delta_encoder_left = encoder_left_current - encoder_left_prev;
        double delta_encoder_right = encoder_right_current - encoder_right_prev;
        encoder_left_prev = encoder_left_current;
        encoder_right_prev = encoder_right_current;


        // Расчёт изменений расстояния для каждого колеса
        double delta_s_l = delta_encoder_left * wheel_radius;
        double delta_s_r = delta_encoder_right * wheel_radius;

        // Расчет сколько было пройдено средней точкой
        double delta_s = (delta_s_r + delta_s_l)/2;
        double delta_tetha = (delta_s_r - delta_s_l) / wheel_base; // угол поворота вокруг вертикальной оси  очередность важна

        x += delta_s * cos(theta + delta_tetha/2.0);
        y += delta_s * sin(theta + delta_tetha/2.0);
        theta += delta_tetha;


        // Обновление матрицы ковариации
        Eigen::MatrixXd J_h (3,2);
        J_h << wheel_radius/2 * cos(theta),wheel_radius/2 * cos(theta),
            wheel_radius/2 * sin(theta),wheel_radius/2 * sin(theta),
            -wheel_radius/wheel_base, wheel_radius/wheel_base;

        // J_h(0,0) = wheel_radius/2 * cos(theta);
        // J_h(0,1) = wheel_radius/2 * cos(theta);
        // J_h(1,0) = wheel_radius/2 * sin(theta);
        // J_h(1,1) = wheel_radius/2 * sin(theta);
        // J_h(2,0) = -wheel_radius/wheel_base;
        // J_h(2,1) = wheel_radius/wheel_base;

        P = P + Q + (J_h * R) * J_h.transpose();


        // Публикация сообщения одометрии
        publishOdom(dt, delta_s,delta_tetha);
        publishTF();


    }
    void publishOdom(double dt, double delta_s, double delta_tetha){
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0,0, theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Заполнение матрица ковариации pose (дисперсии??)
        for (int i = 0; i < 2; ++i){
            for(int j = 0; j < 2; ++j){
                odom_msg.pose.covariance[i *6 + j] = P(i,j);
            }

        }
        odom_msg.pose.covariance[5 *6 + 5] = P(2,2);
        // Заполнение оставшихся элементов нулями (для 3D)
        odom_msg.pose.covariance[2 *6 + 2] = 1e-9; // z
        odom_msg.pose.covariance[3 *6 + 3] = 1e-9; // roll
        odom_msg.pose.covariance[4 *6 + 4] = 1e-9; // pitch

        //РАСЧЁТ СКОРОСТЕЙ
        double v_x = (dt > 0) ? (delta_s * cos(theta)) / dt :0.0;
        double v_y = (dt > 0) ? (delta_s * sin(theta)) / dt :0.0;
        double omega_z = (dt > 0) ? delta_tetha / dt :0.0;

        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.linear.y = v_y;
        odom_msg.twist.twist.angular.z = omega_z;

        // Заполнение матрицы ковариации для скорости
        odom_msg.twist.covariance[0] = P(0,0)/pow(dt,2);
        odom_msg.twist.covariance[7] = P(1,1)/pow(dt,2);
        odom_msg.twist.covariance[14] =1e-9;
        odom_msg.twist.covariance[21] =1e-9;
        odom_msg.twist.covariance[28] =1e-9;
        odom_msg.twist.covariance[35] = P(2,2)/pow(dt,2);

        odom_pub->publish(odom_msg);


    }
    void publishTF(){
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";


        // параметры перемещения
        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0,0, theta);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster-> sendTransform(transform_stamped);
    }



private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    double wheel_radius;
    double wheel_width;
    double wheel_base;
    double tiks_per_revolution;
    double encoder_right_prev;
    double encoder_left_prev;
    double x;
    double y;
    double theta;
    rclcpp::Time last_time;
    Eigen::Matrix3d P; // Матрица ковариации состояния
    Eigen::Matrix3d Q; // Матрица кофариации шума измерений
    Eigen::Matrix2d R; // Матрица ковариации шума процесса





};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotOdometry>());
    rclcpp::shutdown();
    return 0;


}
