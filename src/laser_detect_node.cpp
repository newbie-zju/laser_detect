#include "laser_detect/laser_detect.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"laser_detect_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting laser_detect_node...");
    
    LaserDetect laserdetect(nh);
    
    ros::spin();
    return 0;
}