#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "rosnao_bridge/image_subscriber.hpp"

rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga1 = nullptr;
rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga2 = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga1 = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga2 = nullptr;
rosnao::ImageSubscriber<rosnao::k4VGA> *sub_4vga1 = nullptr;
rosnao::ImageSubscriber<rosnao::k4VGA> *sub_4vga2 = nullptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosnao_image_relay");
    ros::NodeHandle nh;

    if (argc != 7)
    {
        std::cerr << "Wrong arguments for IMAGE_RELAY. Usage: shm_id2, shm_id2, res {1=QVGA, 2=VGA, 3=4VGA}, topic1, topic2, frame_id" << std::endl;
        return 1;
    }

    std::cout << ros::this_node::getName()
              << ": shm_id1[" << argv[1]
              << ": shm_id2[" << argv[2]
              << "] res[" << argv[3]
              << "] topic1[" << argv[4]
              << "] topic2[" << argv[5]
              << "] frame_id[" << argv[6]
              << "]" << std::endl;

    const std::string shm_id1 = argv[1];
    const std::string shm_id2 = argv[2];
    const int res = std::stoi(argv[3]);
    const std::string topic1 = argv[4];
    const std::string topic2 = argv[5];
    const std::string frame_id = argv[6];

    if (res == rosnao::kQVGA){
        sub_qvga1 = new rosnao::ImageSubscriber<rosnao::kQVGA>(shm_id1, frame_id);
        sub_qvga2 = new rosnao::ImageSubscriber<rosnao::kQVGA>(shm_id2, frame_id);
    }
    else if (res == rosnao::kVGA){
        sub_vga1 = new rosnao::ImageSubscriber<rosnao::kVGA>(shm_id1, frame_id);
        sub_vga2 = new rosnao::ImageSubscriber<rosnao::kVGA>(shm_id2, frame_id);
    }
    else if (res == rosnao::k4VGA){
        sub_4vga1 = new rosnao::ImageSubscriber<rosnao::k4VGA>(shm_id1, frame_id);
        sub_4vga2 = new rosnao::ImageSubscriber<rosnao::k4VGA>(shm_id2, frame_id);
    }
    else assert(false); // res must be 1 (QVGA) or 2 (VGA)

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub1 = it.advertise(topic1, 1);
    image_transport::Publisher pub2 = it.advertise(topic2, 1);

    while (ros::ok())
    {
        /* 
        std::pair<cv::Mat, bool> p;
        if (res == rosnao::kVGA)
            p = sub_vga->getCvMat();
        else if (res == rosnao::kQVGA)
            p = sub_qvga->getCvMat(); 
        
        if (p.second == false)
            continue; // don't publish anything if nothing is received

        cv::imshow("test", p.first);
        cv::waitKey(3);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", p.first).toImageMsg();
        pub.publish(msg);
        */
        std::pair<sensor_msgs::ImageConstPtr, bool> p1;
        std::pair<sensor_msgs::ImageConstPtr, bool> p2;
        if (res == rosnao::kQVGA){
            p1 = sub_qvga1->getImageMsg();
            p2 = sub_qvga2->getImageMsg();
        }
        else if (res == rosnao::kVGA){
            p1 = sub_vga1->getImageMsg();
            p2 = sub_vga2->getImageMsg();
        }
        else if (res == rosnao::k4VGA){
            p1 = sub_4vga1->getImageMsg();
            p2 = sub_4vga2->getImageMsg();
        }
        if (p1.second == false || p2.second == false)
            continue; // don't publish anything if nothing is received
        
        pub1.publish(p1.first);
        pub2.publish(p2.first);

        ros::spinOnce();
    }

    if (res == rosnao::kQVGA){
        delete sub_qvga1;
        delete sub_qvga2;
    }
    else if (res == rosnao::kVGA){
        delete sub_vga1;
        delete sub_vga1;
    }
    else if (res == rosnao::k4VGA){
        delete sub_4vga1;
        delete sub_4vga2;
    }

    ros::spin();
    return 0;
}