#include "rosnao_wrapper/IMU_publisher.hpp"
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <chrono>  // For std::chrono
#include <thread>  // For std::this_thread::sleep_for

_def_interrupt;

int main(int argc, char** argv){
    _init_interrupt;

    AL::ALMotionProxy proxy("192.168.26.156");
    const double loopRateHz = 10.0;  // Target loop rate in Hz
    const auto loopDuration = std::chrono::milliseconds(static_cast<int>(1000.0 / loopRateHz));

    while(_no_interrupt){
        auto loopStart = std::chrono::steady_clock::now();

        bool useSensorValues = false;
        std::vector<float> result = proxy.getRobotPosition(useSensorValues);
        std::vector<float> result2 = proxy.getRobotVelocity();
        std::cout << "Robot position is: " << result << std::endl;
        std::cout << "Robot velocity is: " << result2 << std::endl;

        // Calculate elapsed time and sleep if needed to maintain loop rate
        auto loopEnd = std::chrono::steady_clock::now();
        auto elapsed = loopEnd - loopStart;
        if (elapsed < loopDuration) {
            std::this_thread::sleep_for(loopDuration - elapsed);
        }
    }

    _uninstall_interrupt;
    return 0;
}