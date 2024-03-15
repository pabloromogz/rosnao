#include "alproxies/almemoryproxy.h"
#include "rosnao_wrapper/common.hpp"

_def_interrupt;

int main(int argc, char** argv){

    _init_interrupt;
    AL::ALMemoryProxy proxy("192.168.225.156");
    AL::ALValue ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z, ang_x, ang_y, ang_z;
    while(_no_interrupt){
        ang_vel_x = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
        ang_vel_y = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
        ang_vel_z = proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
        acc_x = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
        acc_y = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
        acc_z = proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
        ang_x = proxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
        ang_y = proxy.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
        ang_z = proxy.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
        std::cout << "ANGULAR VELOCITY:\tx: " << ang_vel_x << "\ty: " << ang_vel_y << "\tz: " << ang_vel_z << std::endl;
        std::cout << "ACCELERATION\tx: " << acc_x << "\ty: " << acc_y << "\tz: " << acc_z << std::endl;
        std::cout << "ANGLES:\tx: " << ang_x << "\ty: " << ang_y << "\tz: " << ang_z << std::endl << std::endl;
    }

    _uninstall_interrupt;
    return 0;
}