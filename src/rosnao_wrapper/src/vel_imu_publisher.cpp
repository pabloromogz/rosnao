#include "rosnao_wrapper/vel_imu_publisher.hpp"
#include <thread>
#include <chrono>

rosnao::velIMUPublisher *pub_vel_imu = nullptr;

_def_interrupt;

int main(int argc, char** argv){

    if (argc != 3){
        std::cerr << "Wrong arguments for VEL_IMU_PUB. Usage: nao_ip, shm_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        //
        return 1;
    }
    _init_interrupt;

    const std::string nao_ip = argv[1];
    const std::string shm_id = argv[2];

    std::cout << "VEL_IMU_PUBLISHER: NAO_IP[" << argv[1]
              << "] shm_id[" << argv[2] 
              << "]" << std::endl;

    pub_vel_imu = new rosnao::velIMUPublisher(nao_ip, shm_id);

    while(_no_interrupt){
        pub_vel_imu->pub();
    }

    delete pub_vel_imu;
    _uninstall_interrupt;
    return 0;
}