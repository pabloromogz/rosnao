#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("Received Image", img);
        cv::waitKey(30); // Adjust this value as needed
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "webcam_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("cam0/image_raw", 1, imageCallback);

    ros::spin();

    return 0;
}