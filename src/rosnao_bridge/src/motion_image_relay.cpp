#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "rosnao_bridge/image_subscriber.hpp"
#include "rosnao_bridge/motion.hpp"

rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga = nullptr;

int main(int argc, char **argv){

    ros::init(argc, argv, "rosnao_motion_image_relay");
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

    rosnao::Motion motion(shm_id); // stiffens the robot's joints and makes the robot assume a walking posture

    motion.setAngle(rosnao::HeadYaw, 1.57, 0.05, false); // rotate head to 90deg left (non blocking)
    motion.setAngle(rosnao::HeadYaw, -1.57, 0.1, false); // rotate head to 90deg right, faster (non blocking, doesn't care if the head reached 90deg left)

    while (ros::ok())
    {
        std::pair<sensor_msgs::ImageConstPtr, bool> p;
        if (res == rosnao::kVGA)
            p = sub_vga->getImageMsg();
        else if (res == rosnao::kQVGA)
            p = sub_qvga->getImageMsg();     

        if (p.second == false)
            continue; // don't publish anything if nothing is received
        
        pub.publish(p.first);

        ros::spinOnce();
    }

    motion.rest(); // tells the robot to rest to prevent robot overheating

    if (res == rosnao::kVGA)
        delete sub_vga;
    else if (res == rosnao::kQVGA)
        delete sub_qvga;

    ros::spin();
    return 0;
}