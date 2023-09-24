#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/cam0/image_raw", 1);

    cv::VideoCapture cap(0); // Open the default camera (usually webcam)

    if (!cap.isOpened()) // Check if we succeeded
    {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    ros::Rate loop_rate(10); // Publish at 1 Hz (1 image per second)

    while (ros::ok())
    {
        cv::Mat frame;
        cap >> frame; // Capture a frame from the camera

        if (!frame.empty())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}