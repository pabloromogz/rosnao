#include "rosnao_common/vel_imu.hpp"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#pragma once

namespace rosnao{
    class VelIMUSubscriber{
    private:
        using _vel_imu_struct = transport::SHMVelIMU;
        _vel_imu_struct *shm_vel_imu;
        boost::interprocess::mapped_region region;
        sensor_msgs::ImuPtr imu_msg;
        geometry_msgs::TwistStampedPtr vel_msg;
        std::string shm_id;
        uint32_t seq=0;

    public:
        VelIMUSubscriber &operator=(const VelIMUSubscriber &) = delete;
        VelIMUSubscriber(const VelIMUSubscriber &) = delete;

        VelIMUSubscriber(const std::string &shm_id, const std::string &frame_id) : shm_id(shm_id) {
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
                    shm_vel_imu = static_cast<_vel_imu_struct *>(addr);
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
            vel_msg = boost::make_shared<geometry_msgs::TwistStamped>();
            vel_msg->header.frame_id = frame_id;
        }

        ~VelIMUSubscriber(){ boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        // returns the imu data as a ImuConstPtr.
        // returns true if there is new data rom the stream, false otherwise
        std::pair<sensor_msgs::ImuConstPtr, bool> getIMUMessage(){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_vel_imu->mutex);
            if (shm_vel_imu->seq == seq){
                // std::cout << "Vaya por dios:" << shm_vel_imu->seq << std::endl;
                return std::make_pair(imu_msg, false);
            }
            auto &header = imu_msg->header;
            header.seq = seq = shm_vel_imu->seq;
            header.stamp = ros::Time::now();

            auto &ang = imu_msg->orientation;
            ang.z = shm_vel_imu->imu_ang_z;
            return std::make_pair(imu_msg, true);
        }

        std::pair<geometry_msgs::TwistStampedConstPtr, bool> getVelMessage(){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_vel_imu->mutex);
            if (shm_vel_imu->seq == seq){
                // std::cout << "Vaya por dios:" << shm_vel_imu->seq << std::endl;
                return std::make_pair(vel_msg, false);
            }
            auto &header = vel_msg->header;
            header.seq = seq = shm_vel_imu->seq;
            header.stamp = ros::Time::now();

            auto &lin = vel_msg->twist.linear;
            lin.x = shm_vel_imu->lin_vel_x;
            lin.y = shm_vel_imu->lin_vel_y;

            auto &ang = vel_msg->twist.angular;
            ang.z = shm_vel_imu->ang_vel_z;
            return std::make_pair(vel_msg, true);
        }
    };
}