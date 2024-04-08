#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "rosnao_bridge/imu_subscriber.hpp"

rosnao::IMUSubscriber *sub_imu = nullptr;

int main(int argc, char** argv){

    ros::init(argc, argv, "rosnao_imu_relay");
    ros::NodeHandle nh;

    if (argc != 4){
        std::cerr << "Wrong arguments for IMU_RELAY. Usage: shm_id, topic, frame_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        //
        return 1;
    }

    std::cout << "shm_id[" << argv[1]
              << "] topic[" << argv[2]
              << "] frame_id[" << argv[3]
              << "]" << std::endl;

    const std::string shm_id = argv[1];
    const std::string topic = argv[2];
    const std::string frame_id = argv[3];

    sub_imu = new rosnao::IMUSubscriber(shm_id, frame_id);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(topic, 1);

    ros::Rate loop_rate(3);

    while (ros::ok()){
        loop_rate.sleep();
        std::pair<sensor_msgs::ImuConstPtr, bool> p = sub_imu->getIMUMessage();
        // std::cout << "yay" << std::endl;

        // msg.header.stamp = ros::Time::now();
        // msg.header.frame_id = frame_id;

        // msg.angular_velocity.x = ang_vel_x;
        // msg.angular_velocity.y = ang_vel_y;
        // msg.angular_velocity.z = ang_vel_z;

        // msg.linear_acceleration.x = acc_x;
        // msg.linear_acceleration.y = acc_y;
        // msg.linear_acceleration.z = acc_z;

        // msg.orientation.x = ang_x;
        // msg.orientation.y = ang_y;
        // msg.orientation.z = ang_z;
        // msg.orientation.w = 1.0;  // Assuming static orientation (no rotation)

        // Publish the IMU message
        if (p.second==false) continue;
        imu_pub.publish(p.first);
        ros::spinOnce();
    }

    delete sub_imu;
    ros::spin();
    return 0;
}
