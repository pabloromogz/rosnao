#include "rosnao_wrapper/IMU_publisher.hpp"
#include <alproxies/alsensorsproxy.h>
#include <alproxies/almotionproxy.h>

_def_interrupt;

int main(int argc, char** argv){
    _init_interrupt;

    AL::ALMemoryProxy proxy("192.168.26.156");
    // AL::ALMotionProxy proxy2("192.168.90.156");
    // AL::ALSensorsProxy proxy("192.168.90.156");
    std::vector<std::string> key_list = {"Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                            "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value"};

    // // proxy.subscribe("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
    double prev=0;
    int count=0;
    // while(_no_interrupt){
    // double mean_velx=0, mean_vely=0, mean_velz=0, mean_accx=0, mean_accy=0, mean_accz=0;
    std::vector<double> velsx, velsy, velsz, accsx, accsy, accsz;
    while (count<1000 && _no_interrupt){ 
        AL::ALValue angles_arr = proxy.getListData(key_list);
        double velx = angles_arr[0], vely=angles_arr[1], velz=angles_arr[2];
        double accx = angles_arr[3], accy=angles_arr[4], accz=angles_arr[5];
        if (velx==prev) continue;
        prev = velx;
        count++;
        // mean_velx+=x; mean_vely+=y; mean_velz+=z;
        // mean_accx+=x; mean_accy+=y; mean_accz+=z;
        velsx.push_back(velx); velsy.push_back(vely); velsz.push_back(velz);
        accsx.push_back(accx); accsy.push_back(accy); accsz.push_back(accz);
        std::cout << "ANGULAR VELOCITY:\tx: " << angles_arr[0] << "\ty: " << angles_arr[1] << "\tz: " << angles_arr[2] << std::endl;
        std::cout << "ACCELERATION\t\tx: " << angles_arr[3] << "\ty: " << angles_arr[4] << "\tz: " << angles_arr[5] << std::endl;
        std::cout << "ANGLES:\t\tx: " << angles_arr[6]  << "\ty: " << angles_arr[7] << "\tz: " << angles_arr[8] << std::endl << std::endl;

        // bool useSensorValues = false;
        // std::vector<float> result = proxy2.getRobotPosition(useSensorValues);
        // std::vector<float> result2 = proxy2.getRobotVelocity();
        // std::cout << "Robot position is: " << result << std::endl;
        // std::cout << "Robot velocity is: " << result2 << std::endl;
        // for (auto name: proxy.getOutputNames()) std::cout << name << std::endl;
        // std::cout << proxy.getMyPeriod("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value") << std::endl;
    }
    double mean_velx=0, mean_vely=0, mean_velz=0, mean_accx=0, mean_accy=0, mean_accz=0;
    for (int i=0; i<1000; i++){
        mean_velx+=velsx[i]; mean_vely+=velsy[i]; mean_velz+=velsz[i];
        mean_accx+=accsx[i]; mean_accy+=accsy[i]; mean_accz+=accsz[i];
    }
    mean_velx/=1000; mean_vely/=1000; mean_velz/=1000;
    mean_accx/=1000; mean_accy/=1000; mean_accz/=1000;

    double var_velx=0, var_vely=0, var_velz=0, var_accx=0, var_accy=0, var_accz=0;
    for (int i=0; i<1000; i++){
        var_velx += std::pow(velsx[i] - mean_velx, 2);
        var_vely += std::pow(velsy[i] - mean_vely, 2);
        var_velz += std::pow(velsz[i] - mean_velz, 2);
        var_accx += std::pow(accsx[i] - mean_accx, 2);
        var_accy += std::pow(accsy[i] - mean_accy, 2);
        var_accz += std::pow(accsz[i] - mean_accz, 2);
    }
    var_velx/=1000; var_vely/=1000; var_velz/=1000;
    var_accx/=1000; var_accy/=1000; var_accz/=1000;

    std::cout << "ANG VEL MEANS: X: " << mean_velx << "\tY: " << mean_vely << "\tZ: " << mean_velz << std::endl;
    std::cout << "LIN ACC MEANS: X: " << mean_accx << "\tY: " << mean_accy << "\tZ: " << mean_accz << std::endl;

    std::cout << "ANG VEL VARIANCES: X: " << var_velx << "\tY: " << var_vely << "\tZ: " << var_velz << std::endl;
    std::cout << "LIN ACC VARIANCES: X: " << var_accx << "\tY: " << var_accy << "\tZ: " << var_accz << std::endl;

    // while (_no_interrupt){
    //     AL::ALValue val = proxy.getTimestamp("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
    //     std::cout << "Value: " << val[0] << "\tTimestamp1: " << val[1] << "\tTimestamp2: " << val[2] << std::endl;
    // }
    _uninstall_interrupt;
    return 0;
}