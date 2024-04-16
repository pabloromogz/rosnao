#include "rosnao_bridge/estimator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "extended_kalman_filter");
    EstimatorEKF ekf;
    ros::Rate loop_rate(10);  // State prediction at 10 Hz

    while (ros::ok()) {
        ekf.predict(1.0/loop_rate.expectedCycleTime().toSec());  // Prediction step
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}