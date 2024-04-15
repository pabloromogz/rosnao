#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "rosnao_bridge/vel_imu_subscriber.hpp"

rosnao::VelIMUSubscriber *sub_vel_imu = nullptr;

int main(int argc, char** argv){

    ros::init(argc, argv, "rosnao_vel_imu_relay");
    ros::NodeHandle nh;

    if (argc != 5){
        std::cerr << "Wrong arguments for VEL_IMU_RELAY. Usage: shm_id, vel_topic, imu_topic, frame_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        //
        return 1;
    }

    std::cout << "shm_id[" << argv[1]
              << "] vel_topic[" << argv[2]
              << "] imu_topic[" << argv[3]
              << "] frame_id[" << argv[4]
              << "]" << std::endl;

    const std::string shm_id = argv[1];
    const std::string vel_topic = argv[2];
    const std::string imu_topic = argv[3];
    const std::string frame_id = argv[4];

    sub_vel_imu = new rosnao::VelIMUSubscriber(shm_id, frame_id);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);

    ros::Duration imu_period(1.0 / 3.0);
    ros::Duration vel_period(1.0 / 10.0);
    ros::Time current_time = ros::Time::now();
    ros::Time last_imu_publish_time = current_time;
    ros::Time last_vel_publish_time = current_time;

    while (ros::ok()){
        current_time = ros::Time::now();
        if ((current_time - last_imu_publish_time) >= imu_period) {
            std::pair<sensor_msgs::ImuConstPtr, bool> p_ang = sub_vel_imu->getIMUMessage();
            if (p_ang.second) {
                imu_pub.publish(p_ang.first);
                last_imu_publish_time = current_time;
            }
        }
        
        if ((current_time - last_vel_publish_time) >= vel_period){
            std::pair<geometry_msgs::TwistStampedConstPtr, bool> p_vel = sub_vel_imu->getVelMessage();
            if (p_vel.second) {
                vel_pub.publish(p_vel.first);
                last_vel_publish_time = current_time;
            }
        }
        ros::spinOnce();
    }

    delete sub_vel_imu;
    ros::spin();
    return 0;
}
