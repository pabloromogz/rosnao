#include <iostream>
#include "rosnao_wrapper/common.hpp"
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <chrono>  // For std::chrono
#include <thread>  // For std::this_thread::sleep_for

_def_interrupt;

int main(int argc, char** argv){
    _init_interrupt;

    AL::ALMotionProxy motion("192.168.26.156");
    AL::ALMemoryProxy imu("192.168.26.156");
    const double loopRateHz = 10.0;  // Target loop rate in Hz
    const double loopDurationSec = 1.0 / loopRateHz; // Duration of each loop iteration in seconds
    auto before = std::chrono::steady_clock::now();
    double pos_x=0, pos_y=0, angle_z=0;
    double vel_x, vel_y, vel_z;
    double angle_bias = imu.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");

    while(_no_interrupt){
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - before).count();
        if (elapsed < loopDurationSec) {
            continue;
        }
        before = now;

        double angle = imu.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
        angle -= angle_bias;
        std::vector<float> velocities = motion.getRobotVelocity();
        std::cout << "Robot velocity is: " << velocities << std::endl;
        std::cout << "Robot angle is: " << angle << std::endl;

        pos_x += vel_x * elapsed;
        pos_y += vel_y * elapsed;

        vel_x = velocities[0] * cos(angle_z) - velocities[1] * sin(angle_z);
        vel_y = velocities[0] * sin(angle_z) + velocities[1] * cos(angle_z);
        angle_z += vel_z * elapsed;
        vel_z = velocities[2];
        
        vel_x = vel_x>0.005? vel_x : 0;
        vel_y = vel_y>0.005? vel_y : 0;
        vel_z = vel_z>0.005? vel_z : 0;

        std::cout << "State:\tX: " << pos_x << "\tY: " << pos_y << "\tTheta: " << angle_z << std::endl;
        std::cout << "Vels:\tX: " << vel_x << "\tY: " << vel_y << "\tTheta: " << vel_z << std::endl;
    }

    _uninstall_interrupt;
    return 0;
}