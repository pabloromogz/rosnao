#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <signal.h>
#include "rosnao_bridge/image_subscriber.hpp"
#include "rosnao_bridge/motion.hpp"
#include "rosnao_common/common.hpp"

rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga = nullptr;

int main(int argc, char **argv){
    ros::init(argc, argv, "rosnao_image_relay");
    ros::NodeHandle nh;

    if (argc != 5)
    {
        std::cerr << "Wrong arguments for IMAGE_RELAY. Usage: shm_id, res {1=QVGA, 2=VGA}, topic, frame_id" << std::endl;
        return 1;
    }

    std::cout << ros::this_node::getName()
              << ": shm_id[" << argv[1]
              << "] res[" << argv[2]
              << "] topic[" << argv[3]
              << "] frame_id[" << argv[4]
              << "]" << std::endl;

    const std::string shm_id = argv[1];
    const int res = std::stoi(argv[2]);
    const std::string topic = argv[3];
    const std::string frame_id = argv[4];

    if (res == rosnao::kVGA)
        sub_vga = new rosnao::ImageSubscriber<rosnao::kVGA>(shm_id, frame_id);
    else if (res == rosnao::kQVGA)
        sub_qvga = new rosnao::ImageSubscriber<rosnao::kQVGA>(shm_id, frame_id);
    else
        assert(false); // res must be 1 (QVGA) or 2 (VGA)

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);

    std::cout << "TELEOP: Waiting for NAO to stand up..." << std::endl;

    rosnao::Motion motion(shm_id); // stiffens the robot's joints and makes the robot assume a walking posture
    float x_nvel = 0, y_nvel = 0, yaw_nvel = 0;

    std::cout << "================== TELEOPERATE NAO ===================" << std::endl;
    std::cout << "|  w,x to move forward(+x) or backward(-x)           |" << std::endl;
    std::cout << "|  a,d to strafe leftward(+y) or rightward(-y)       |" << std::endl;
    std::cout << "|  q,e to rotate left or right                       |" << std::endl;
    std::cout << "|  s   to stop all motion                            |" << std::endl;
    std::cout << "|  all other keys stop the teleoperation             |" << std::endl;
    std::cout << "======================================================" << std::endl;

    // Set terminal to raw mode
    std::system("stty raw");

    while (ros::ok()){

        std::pair<sensor_msgs::ImageConstPtr, bool> p;
        if (res == rosnao::kVGA)
            p = sub_vga->getImageMsg();
        else if (res == rosnao::kQVGA)
            p = sub_qvga->getImageMsg();     

        if (p.second == false)
            continue; // don't publish anything if nothing is received
        
        pub.publish(p.first);

        try{
            char c = getchar();
            if (c == 's')
            {
                x_nvel = y_nvel = yaw_nvel = 0;
            }
            else if (c == 'w')
            {
                x_nvel += 0.1;
                if (x_nvel > 1)
                    x_nvel = 1;
            }
            else if (c == 'a')
            {
                y_nvel += 0.1;
                if (y_nvel > 1)
                    y_nvel = 1;
            }
            else if (c == 'd')
            {
                y_nvel -= 0.1;
                if (y_nvel < -1)
                    y_nvel = -1;
            }
            else if (c == 'x')
            {
                x_nvel -= 0.1;
                if (x_nvel < -1)
                    x_nvel = -1;
            }
            else if (c == 'q')
            {
                yaw_nvel += 0.1;
                if (yaw_nvel > 1)
                    yaw_nvel = 1;
            }
            else if (c == 'e')
            {
                yaw_nvel -= 0.1;
                if (yaw_nvel < -1)
                    yaw_nvel = -1;
            }
            else
                break;

            motion.moveToward(x_nvel, y_nvel, yaw_nvel);

            std::cout << std::fixed
                      << "\bInput[" << c
                      << "]  NormVel( X:" << std::setw(5) << std::setprecision(2) << x_nvel
                      << ", Y:" << std::setw(5) << std::setprecision(2) << y_nvel
                      << ", Yaw:" << std::setw(5) << std::setprecision(2) << yaw_nvel
                      << " )      \r";
        }
        catch (...){
            std::cout << "Exception caught. Did the robot fall down?" << std::endl;
            break;
        }
        ros::spinOnce();
    }

    if (res == rosnao::kVGA)
        delete sub_vga;
    else if (res == rosnao::kQVGA)
        delete sub_qvga;
    
    std::cout << std::endl;

    // Reset terminal
    std::system("stty cooked");
    std::system("stty sane");

    std::cout << "TELEOP: Waiting for NAO to rest..." << std::endl;
    motion.rest(); // tells the robot to rest to prevent robot overheating
    std::cout << "TELEOP: Done!" << std::endl;

    ros::spin();
    return 0;
}