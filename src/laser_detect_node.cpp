#include "laser_detect/laser_detect.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"laser_detect_node");
    ros::NodeHandle nh, private_nh;
    ROS_INFO("Starting laser_detect_node...");
    
    LaserDetect laserdetect(nh,private_nh);
    
    ros::spin();
    return 0;
}