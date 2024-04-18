#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>

class Controller {
public:
    Controller() {
        nh_ = ros::NodeHandle("~");

        odom_sub_ = nh_.subscribe("/odom", 1, &Controller::odomCallback, this);
        // goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &Controller::goalCallback, this);
        goal_sub_ = nh_.subscribe("/clicked_point", 1, &Controller::goalCallback, this);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        max_linear_velocity_ = 0.05;  // Maximum linear velocity in m/s
        max_angular_velocity_ = 0.05;  // Maximum angular velocity in rad/s

        // goal_pose_ = current_pose_;
        goal_point_ = current_pose_.position;
        reached_ = true;
    }

    bool reached(){
        return reached_;
    };

    void update_vels(){
        geometry_msgs::Twist cmd_vel;
        // double diff_x = goal_pose_.position.x - current_pose_.position.x;
        // double diff_y = goal_pose_.position.y - current_pose_.position.y;
        double diff_x = goal_point_.x - current_pose_.position.x;
        double diff_y = goal_point_.y - current_pose_.position.y;
        double diff_theta = atan2(diff_y, diff_x) - tf::getYaw(current_pose_.orientation);
        diff_theta = limit_angle(diff_theta);
        // std::cout << "DELTA X: " << diff_x << "\tDELTA Y: " << diff_y << "\tANGLE DIFF: " << diff_theta << std::endl;
        // std::cout << "ROBOT X DIFF: " << distance_() * cos(diff_theta)<< "\tROBOT Y DIFF: " << distance_() * sin(diff_theta) << "\tAROBOT ANGLE DIFF: " << diff_theta << std::endl;
        cmd_vel.linear.x = constrain_vel(0.1*distance_() * cos(diff_theta), max_linear_velocity_);
        cmd_vel.linear.y = constrain_vel(0.1*distance_() * sin(diff_theta), max_linear_velocity_);
        cmd_vel.angular.z = constrain_vel(0.1*diff_theta, max_angular_velocity_);
        cmd_vel_pub_.publish(cmd_vel);
        update_state_();
    }

    double limit_angle(double angle){
        double result = fmod(angle + M_PI, M_PI*2);
        return result >= 0? result - M_PI : result + M_PI;
    }

    double constrain_vel(double command, double constraint){
        return command < 0 ? -std::min(abs(command), constraint) : std::min(abs(command), constraint);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Pose current_pose_;
    // geometry_msgs::Pose goal_pose_;
    geometry_msgs::Point goal_point_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    bool reached_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
    }

    // void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //     goal_pose_ = msg->pose;
    //     update_state_();
    // }

    void goalCallback(const geometry_msgs::PointStampedConstPtr& msg) {
        goal_point_ = msg->point;
        update_state_();
    }

    // double distance_(){
    //     return sqrt(pow(current_pose_.position.x - goal_pose_.position.x, 2) + pow(current_pose_.position.y - goal_pose_.position.y, 2));
    // }

    double distance_(){
        return sqrt(pow(current_pose_.position.x - goal_point_.x, 2) + pow(current_pose_.position.y - goal_point_.y, 2));
    }

    void update_state_(){
        if (distance_() >= 0.1) reached_ = false;
        else {
            reached_ = true;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x=0; cmd_vel.linear.y=0; cmd_vel.angular.z=0;
            cmd_vel_pub_.publish(cmd_vel);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_navigation_node");
    Controller controller_node;
    ros::Rate controller_rate(2);
    while(ros::ok()){
        if (!controller_node.reached()){
            controller_node.update_vels();
        }
        ros::spinOnce();
        controller_rate.sleep();
    }
    ros::spin();
    return 0;
}