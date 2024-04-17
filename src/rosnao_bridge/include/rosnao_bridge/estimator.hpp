#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class EstimatorEKF {
public:
    EstimatorEKF() : nh_("~") {
        X_.setZero();
        U_.setZero();
        P_.setIdentity();
        P_ *= 1e3;
        Q_ << 0.0001, 0, 0,
              0, 0.0001,0,
              0, 0, 0.01;

        sub_robot_vel_ = nh_.subscribe("/robot_vel", 1, &EstimatorEKF::robotVelCallBack, this);
        sub_imu_ = nh_.subscribe("/imu0", 1, &EstimatorEKF::IMUCallback, this);
        sub_slam_pose_ = nh_.subscribe("/orb_slam3/camera_pose", 1, &EstimatorEKF::slamPoseCallback, this);
        pub_filtered_pose_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
    }

    void predict (const double& elapsed){
        Eigen::Matrix<double, 6, 6> F_;
        Eigen::Matrix<double, 6, 3> W_;
        double theta = X_(2);

        F_ << 1, 0, 0, elapsed, 0, 0,
             0, 1, 0, 0, elapsed, 0,
             0, 0, 1, 0, 0, elapsed,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0;
        W_ << 0, 0, 0,
              0, 0, 0,
              0, 0, 0,
              cos(theta), -sin(theta), 0,
              sin(theta), cos(theta), 0,
              0, 0, 1;

        X_ = F_ * X_ + W_ * U_;
        P_ = F_ * P_ * F_.transpose() + W_ * Q_ * W_.transpose();

        publish();
        verbose();
    }

    void correctPose(const geometry_msgs::PoseStampedConstPtr& msg){
        Eigen::Matrix<double, 2, 6> H_slam_;
        H_slam_ << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0;
        Eigen::Matrix2d R_slam_;
        R_slam_ << 0.5, 0,
                   0, 0.5;

        Eigen::Matrix<double, 6, 2> K_;
        K_ = P_ * H_slam_.transpose() * (H_slam_ * P_ * H_slam_.transpose() + R_slam_).inverse();

        z_slam_ = {msg->pose.position.x, msg->pose.position.y};
        X_ += K_ * (z_slam_ - H_slam_ * X_);
        P_ -= K_ * H_slam_ * P_;
    }

    void correctTheta(const sensor_msgs::ImuConstPtr& msg){
        Eigen::RowVector<double, 6> H_imu_ = {0, 0, 1, 0, 0, 0};
        double R_imu_ = 0.0000002; //SET value

        Eigen::Vector<double, 6> K_;
        K_ = P_ * H_imu_.transpose() / (H_imu_ * P_ * H_imu_.transpose() + R_imu_);
        
        z_imu_ = msg->orientation.z - imu_offset_ + imu_slam_offset_;
        X_ += K_ * limit_angle(z_imu_ - H_imu_ * X_);
        X_(2) = limit_angle(X_(2));
        P_ -= K_*H_imu_*P_;
    }

    void verbose(){
        std::cout << "Pose\t(" << X_(0) << ", " << X_(1) << ", " << X_(2) << " )" << std::endl;
        std::cout << "Twist\t(" << X_(3) << ", " << X_(4) << ", " << X_(5) << " )" << std::endl;
        std::cout << "SLAM\t(" << z_slam_(0) << ", " << z_slam_(1) << ", " << "---" << " )" << std::endl;
        std::cout << "IMU\t(" << "---" << ", " << "---" << ", " << z_imu_ << " )" << std::endl << std::endl;
    }

    void publish(){
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time::now();
        odom.child_frame_id = "camera";
        geometry_msgs::PoseWithCovariance filtered_pose;
        filtered_pose.pose.position.x = X_(0);
        filtered_pose.pose.position.y = X_(1);
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(0, 0, X_(2));
        filtered_pose.pose.orientation = tf2::toMsg(quaternion_tf2);
        geometry_msgs::TwistWithCovariance filtered_twist;
        filtered_twist.twist.linear.x = X_(3);
        filtered_twist.twist.linear.y = X_(4);
        filtered_twist.twist.angular.z = X_(5);
        odom.pose = filtered_pose;
        odom.twist = filtered_twist;
        pub_filtered_pose_.publish(odom);
    }

    double limit_angle(double angle){
        double result = fmod(angle + M_PI, M_PI*2);
        return result >= 0? result - M_PI : result + M_PI;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_robot_vel_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_slam_pose_;
    ros::Publisher pub_filtered_pose_;

    Eigen::Vector<double, 6> X_;
    Eigen::Matrix<double, 6, 6> P_;
    Eigen::Matrix<double, 3, 3> Q_;
    Eigen::Vector<double, 3> U_;

    double z_imu_= NAN, imu_offset_=0, imu_slam_offset_=0;
    Eigen::Vector2d z_slam_ = {NAN, NAN};

    void robotVelCallBack(const geometry_msgs::TwistStampedConstPtr& msg){
        U_(0) = msg->twist.linear.x;
        U_(1) = msg->twist.linear.y;
        U_(2) = msg->twist.angular.z;
    }

    void IMUCallback(const sensor_msgs::ImuConstPtr& msg){

        static bool init_imu = false;
        if (!init_imu){
            imu_offset_ = msg->orientation.z;
            std::cout << "INITIALISING IMU OFFSET!" << std::endl;
            init_imu = true;
            return;
        }

        correctTheta(msg);
    }

    void slamPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

        static bool init_pose = false;
        if (!init_pose){
            X_(0) = msg->pose.position.x;
            X_(1) = msg->pose.position.y;
            // X_(2) = msg->pose.orientation.z;
            // imu_slam_offset_ = msg->pose.orientation.z;
            std::cout << "INITIALISING EKF POSE TO SLAM ESTIMATE!" << std::endl;
            init_pose = true;
            return;
        }

        correctPose(msg);
    }
};