#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "rosnao_bridge/motion.hpp"

class ControlledMotion {
public:
    ControlledMotion(const std::string& shm_id) : motion_(shm_id) {
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &ControlledMotion::cmdVelCallback, this);
        motion_.wakeUp();
        motion_.moveInit();
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        motion_.move(msg->linear.x, msg->linear.y, msg->angular.z);
    }
    ~ControlledMotion(){
        motion_.rest();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    rosnao::Motion motion_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosnao_test_motion");
    ros::NodeHandle nh;

    if (argc != 2) {
        std::cerr << "Wrong arguments for CONTROL_MOTION. Usage: shm_id" << std::endl;
        return 1;
    }

    const std::string shm_id = argv[1];
    ControlledMotion motion_control(shm_id);
    ros::spin();
    return 0;
}