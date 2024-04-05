#include "alproxies/almemoryproxy.h"
#include <signal.h>
#include <iostream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "rosnao_common/imu.hpp"
#include "rosnao_wrapper/common.hpp"

#ifndef ROSNAO_IMU_PUBLISHER_HPP
#define ROSNAO_IMU_PUBLISHER_HPP

namespace rosnao{

    class IMUPublisher{
    private:
        using _imu_struct = transport::SHMIMU;
        AL::ALMemoryProxy proxy;
        boost::interprocess::mapped_region region; // cannot be deleted (will cause segmentation fault)
        _imu_struct *shm_imu;
        std::string shm_id;
        std::vector<std::string> key_list = {"Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                             "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value"};

        void _fill(const AL::ALValue& angles){
            ++shm_imu->seq;

            auto &shm_data = shm_imu->data;
            for (size_t i=0; i<9; ++i)
                shm_data[i] = angles[i];

            std::cout << "IMU PUBLISHING INTO SHARED MEMORY" << std::endl;
            std::cout << "SEQUENCE NUMBER: " << shm_imu->seq << std::endl;
            std::cout << "ANGULAR VELOCITY:\tx: " << shm_imu->data[0] << "\ty: " << shm_imu->data[1] << "\tz: " << shm_imu->data[2] << std::endl;
            std::cout << "ACCELERATION\tx: " << shm_imu->data[3] << "\ty: " << shm_imu->data[4] << "\tz: " << shm_imu->data[5] << std::endl;
            std::cout << "ANGLES:\tx: " << shm_imu->data[6] << "\ty: " << shm_imu->data[7] << "\tz: " << shm_imu->data[8] << std::endl << std::endl;

            // shm_imu->ang_vel_x = angles[0];
            // shm_imu->ang_vel_y = angles[1];
            // shm_imu->ang_vel_z = angles[2];
            // shm_imu->acc_x = angles[3];
            // shm_imu->acc_y = angles[4];
            // shm_imu->acc_z = angles[5];
            // shm_imu->ang_x = angles[6];
            // shm_imu->ang_y = angles[7];
            // shm_imu->ang_z = angles[8];

            // std::cout << "IMU PUBLISHING INTO SHARED MEMORY" << std::endl;
            // std::cout << "SEQUENCE NUMBER: " << shm_imu->seq << std::endl;
            // std::cout << "ANGULAR VELOCITY:\tx: " << shm_imu->ang_vel_x << "\ty: " << shm_imu->ang_vel_y << "\tz: " << shm_imu->ang_vel_z << std::endl;
            // std::cout << "ACCELERATION\tx: " << shm_imu->acc_x << "\ty: " << shm_imu->acc_y << "\tz: " << shm_imu->acc_z << std::endl;
            // std::cout << "ANGLES:\tx: " << shm_imu->ang_x  << "\ty: " << shm_imu->ang_y << "\tz: " << shm_imu->ang_z << std::endl << std::endl;
        }

    public:
        IMUPublisher(const std::string &ip, const std::string &shm_id) : proxy(ip), shm_id(shm_id) {
            try{
                boost::interprocess::shared_memory_object::remove(shm_id.c_str());

                boost::interprocess::shared_memory_object shm(
                    boost::interprocess::open_or_create,
                    shm_id.c_str(),
                    boost::interprocess::read_write);

                shm.truncate(sizeof(_imu_struct));
                region = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);
                void *addr = region.get_address();
                shm_imu = new (addr)(_imu_struct);
            }
            catch (boost::interprocess::interprocess_exception &ex)
            {
                std::cerr << "Boost Interprocess Exception: " << ex.what() << std::endl;
                boost::interprocess::shared_memory_object::remove(shm_id.c_str());
                delete this;
            }
        }

        ~IMUPublisher(){
            boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        void pub(){
            // std::cout << "Fetching IMU Data" << std::endl;

            // std::vector<AL::ALValue> angles(9);

            AL::ALValue angles_arr = proxy.getListData(key_list);
            // std::cout << "IMU PUBLISHING INTO SHARED MEMORY" << std::endl;
            // std::cout << "ANGULAR VELOCITY:\tx: " << angles_arr[0] << "\ty: " << angles_arr[1] << "\tz: " << angles_arr[2] << std::endl;
            // std::cout << "ACCELERATION\tx: " << angles_arr[3] << "\ty: " << angles_arr[4] << "\tz: " << angles_arr[5] << std::endl;
            // std::cout << "ANGLES:\tx: " << angles_arr[6]  << "\ty: " << angles_arr[7] << "\tz: " << angles_arr[8] << std::endl << std::endl;

            // angles[0] = angles_arr[0];
            // angles[1] = angles_arr[1];
            // angles[2] = angles_arr[2];
            // angles[3] = angles_arr[3];
            // angles[4] = angles_arr[4];
            // angles[5] = angles_arr[5];
            // angles[6] = angles_arr[6];
            // angles[7] = angles_arr[7];
            // angles[8] = angles_arr[8];

            // angles[0] = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
            // angles[1] = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
            // angles[2] = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
            // angles[3] = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
            // angles[4] = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
            // angles[5] = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
            // angles[6] = proxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
            // angles[7] = proxy.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
            // angles[8] = proxy.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");

            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_imu->mutex); // this will release when _fill goes out of scope
            _fill(angles_arr);
            // std::cout << "Published IMU " << std::endl;
        }
    };
}

#endif