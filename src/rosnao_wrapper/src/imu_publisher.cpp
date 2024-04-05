#include "rosnao_wrapper/IMU_publisher.hpp"
#include <thread>
#include <chrono>

rosnao::IMUPublisher *pub_imu = nullptr;

_def_interrupt;

int main(int argc, char** argv){

    if (argc != 3){
        std::cerr << "Wrong arguments for IMU_PUB. Usage: nao_ip, shm_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        //
        return 1;
    }
    _init_interrupt;

    const std::string nao_ip = argv[1];
    const std::string shm_id = argv[2];

    std::cout << "IMU_PUBLISHER: NAO_IP[" << argv[1]
              << "] shm_id[" << argv[2] 
              << "]" << std::endl;

    pub_imu = new rosnao::IMUPublisher(nao_ip, shm_id);

    // AL::ALMemoryProxy proxy(nao_ip);
    // AL::ALValue ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z, ang_x, ang_y, ang_z;

    while(_no_interrupt){
        pub_imu->pub();
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // ang_vel_x = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
        // ang_vel_y = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
        // ang_vel_z = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
        // acc_x = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
        // acc_y = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
        // acc_z = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
        // ang_x = proxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
        // ang_y = proxy.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
        // ang_z = proxy.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
        // std::cout << "ANGULAR VELOCITY:\tx: " << ang_vel_x << "\ty: " << ang_vel_y << "\tz: " << ang_vel_z << std::endl;
        // std::cout << "ACCELERATION\tx: " << acc_x << "\ty: " << acc_y << "\tz: " << acc_z << std::endl;
        // std::cout << "ANGLES:\tx: " << ang_x << "\ty: " << ang_y << "\tz: " << ang_z << std::endl << std::endl;

        // pub_imu->pub();
    }
    delete pub_imu;
    _uninstall_interrupt;
    return 0;
}