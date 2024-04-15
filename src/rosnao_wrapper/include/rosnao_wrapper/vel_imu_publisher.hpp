#include "alproxies/almemoryproxy.h"
#include "alproxies/almotionproxy.h"
#include <signal.h>
#include <iostream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "rosnao_common/vel_imu.hpp"
#include "rosnao_wrapper/common.hpp"
#include "rosnao_common/common.hpp"

#pragma once

namespace rosnao{

    class velIMUPublisher{
    private:
        using _vel_imu_struct = transport::SHMVelIMU;
        AL::ALMemoryProxy imu;
        AL::ALMotionProxy velocities;
        boost::interprocess::mapped_region region; // cannot be deleted (will cause segmentation fault)
        _vel_imu_struct *shm_vel_imu;
        std::string shm_id;
        std::string imu_key = "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value";

        void _fill(const double& lin_vel_x, const double& lin_vel_y, const double& ang_vel_z, const double& imu_ang_z){
            ++shm_vel_imu->seq;
            shm_vel_imu->lin_vel_x = lin_vel_x;
            shm_vel_imu->lin_vel_y = lin_vel_y;
            shm_vel_imu->ang_vel_z = ang_vel_z;
            shm_vel_imu->imu_ang_z = imu_ang_z;

            std::cout << "VELOCITIES AND IMU PUBLISHING INTO SHARED MEMORY" << std::endl;
            std::cout << "SEQUENCE NUMBER: " << shm_vel_imu->seq << std::endl;
            std::cout << "LINEAR VELOCITY:\tx: " << shm_vel_imu->lin_vel_x << "\ty: " << shm_vel_imu->lin_vel_y << std::endl;
            std::cout << "ANGULAR VELOCITY:\ttheta: " << shm_vel_imu->ang_vel_z << std::endl;
            std::cout << "IMU ANGLE:\tx: " << shm_vel_imu->imu_ang_z << std::endl << std::endl;
        }

    public:
        velIMUPublisher(const std::string &ip, const std::string &shm_id) : imu(ip), velocities(ip), shm_id(shm_id) {
            try{
                boost::interprocess::shared_memory_object::remove(shm_id.c_str());

                boost::interprocess::shared_memory_object shm(
                    boost::interprocess::open_or_create,
                    shm_id.c_str(),
                    boost::interprocess::read_write);

                shm.truncate(sizeof(_vel_imu_struct));
                region = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);
                void *addr = region.get_address();
                shm_vel_imu = new (addr)(_vel_imu_struct);
            }
            catch (boost::interprocess::interprocess_exception &ex)
            {
                std::cerr << "Boost Interprocess Exception: " << ex.what() << std::endl;
                boost::interprocess::shared_memory_object::remove(shm_id.c_str());
                delete this;
            }
        }

        ~velIMUPublisher(){
            boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        void pub(){
            AL::ALValue angle_z = imu.getData(imu_key);
            std::vector<float> vels = velocities.getRobotVelocity();

            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_vel_imu->mutex); // this will release when _fill goes out of scope
            _fill(vels[0], vels[1], vels[2], angle_z);
        }
    };
}