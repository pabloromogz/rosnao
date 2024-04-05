#include "rosnao_common/imu.hpp"
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

#ifndef ROSNAO_BRIDGE_IMU_SUBSCRIBER_HPP
#define ROSNAO_BRIDGE_IMU_SUBSCRIBER_HPP

namespace rosnao{
    class IMUSubscriber{
    private:
        using _imu_struct = transport::SHMIMU;
        _imu_struct *shm_imu;
        boost::interprocess::mapped_region region;
        sensor_msgs::ImuPtr imu_msg;
        std::string shm_id;
        uint32_t seq=0;

    public:
        IMUSubscriber &operator=(const IMUSubscriber &) = delete;
        IMUSubscriber(const IMUSubscriber &) = delete;

        IMUSubscriber(const std::string &shm_id, const std::string &frame_id) : shm_id(shm_id) {
            for (int attempt = 0; ros::ok(); ++attempt){
                try{
                    boost::interprocess::shared_memory_object shm(
                        boost::interprocess::open_only,
                        shm_id.c_str(),
                        boost::interprocess::read_write);

                    // Map the whole shared memory in this process
                    region = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);

                    // Get the address of the mapped region
                    void *addr = region.get_address();

                    // Construct the shared structure in memory
                    shm_imu = static_cast<_imu_struct *>(addr);
                    std::cout << "Subscribe attempt " << attempt << " succeeded." << std::endl;
                    break;
                }
                catch (boost::interprocess::interprocess_exception &ex){
                    std::cout << "Failed subscribing attempt " << attempt << ": " << ex.what() << std::endl;
                    ros::Duration(1).sleep();
                }
            }

            // init msg
            imu_msg = boost::make_shared<sensor_msgs::Imu>();
            imu_msg->header.frame_id = frame_id;
        }

        ~IMUSubscriber(){ boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        // returns the imu data as a ImuConstPtr.
        // returns true if there is new data rom the stream, false otherwise
        std::pair<sensor_msgs::ImuConstPtr, bool> getIMUMessage(){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_imu->mutex);
            if (shm_imu->seq == seq){
                std::cout << "Vaya por dios:" << shm_imu->seq << std::endl;
                return std::make_pair(imu_msg, false);
            }
            // std::cout << "DAJE" << std::endl;
            // std::cout << "SEQUENCE NUMBER: " << shm_imu->seq << std::endl;
            // std::cout << "ANGULAR VELOCITY:\tx: " << shm_imu->data[0] << "\ty: " << shm_imu->data[1] << "\tz: " << shm_imu->data[2] << std::endl;
            // std::cout << "ACCELERATION\tx: " << shm_imu->data[3] << "\ty: " << shm_imu->data[4] << "\tz: " << shm_imu->data[5] << std::endl;
            // std::cout << "ANGLES:\tx: " << shm_imu->data[6] << "\ty: " << shm_imu->data[7] << "\tz: " << shm_imu->data[8] << std::endl << std::endl;

            auto &header = imu_msg->header;
            header.seq = seq = shm_imu->seq;
            header.stamp = ros::Time::now();

            auto &ang_vel = imu_msg->angular_velocity;
            ang_vel.x = shm_imu->data[0];
            ang_vel.y = shm_imu->data[1];
            ang_vel.z = shm_imu->data[2];

            auto &lin_acc = imu_msg->linear_acceleration;
            lin_acc.x = shm_imu->data[3];
            lin_acc.y = shm_imu->data[4];
            lin_acc.z = shm_imu->data[5];

            auto &ang = imu_msg->orientation;
            ang.x = shm_imu->data[6];
            ang.y = shm_imu->data[7];
            ang.z = shm_imu->data[8];
            ang.w = 1.0; // Assuming static orientation (no rotation)

            return std::make_pair(imu_msg, true);
        }
    };
}

#endif