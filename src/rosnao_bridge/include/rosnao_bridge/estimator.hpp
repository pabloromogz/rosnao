#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

class EstimatorEKF {
public:
    EstimatorEKF() : nh_("~") {
        X_.setZero();
        U_.setZero();
        P_.setIdentity();
        P_ *= 1e3;
        Q_ << 5, 0, 0, // SET VALUES
              0, 5,0,
              0, 0, 5;

        sub_robot_vel_ = nh_.subscribe("/robot_vel", 1, &EstimatorEKF::robotVelCallBack, this);
        sub_imu_ = nh_.subscribe("/imu0", 1, &EstimatorEKF::IMUCallback, this);
        sub_slam_pose_ = nh_.subscribe("/orb_slam3/camera_pose", 1, &EstimatorEKF::slamPoseCallback, this);
        pub_filtered_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/filtered_pose", 1);
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
    }

    void correctPose(const geometry_msgs::PoseStampedConstPtr& msg){
        Eigen::Matrix<double, 3, 6> H_slam_;
        H_slam_ << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0,
                   0, 0, 1, 0, 0, 0;
        Eigen::Matrix3d R_slam_;
        R_slam_ << 5.0, 0, 0,
                   0, 5.0, 0,
                   0, 0, 5.0;

        Eigen::Matrix<double, 6, 3> K_;
        K_ = P_ * H_slam_.transpose() * (H_slam_ * P_ * H_slam_.transpose() + R_slam_).inverse();

        Eigen::Vector3d z_slam_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z};
        X_ += K_ * (z_slam_ - H_slam_ * X_);
        P_ -= K_ * H_slam_ * P_;
    }

    void correctTheta(const sensor_msgs::ImuConstPtr& msg){
        Eigen::RowVector<double, 6> H_imu_ = {0, 0, 1, 0, 0, 0};
        double R_imu_ = 5.0; //SET value

        Eigen::Vector<double, 6> K_;
        K_ = P_ * H_imu_.transpose() / (H_imu_ * P_ * H_imu_.transpose() + R_imu_);
        
        double z_imu_ = msg->orientation.z;
        X_ += K_ * (z_imu_ - H_imu_ * X_);
        P_ -= K_*H_imu_*P_;
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
    Eigen::Vector<double, 6> U_;

    void robotVelCallBack(const geometry_msgs::TwistStampedConstPtr& msg){
        U_(0) = msg->twist.linear.x;
        U_(1) = msg->twist.linear.y;
        U_(2) = msg->twist.angular.z;
    }

    void IMUCallback(const sensor_msgs::ImuConstPtr& msg){
        correctTheta(msg);
    }

    void slamPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
        correctPose(msg);
    }
};