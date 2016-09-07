#ifndef _LASER_DETECT_H_
#define _LASER_DETECT_H_

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <deque>
#include <stdio.h>
//#include "iarc_tf/ned_world_dynamic.h"
#include <geometry_msgs/Pose.h>

#include "laser_detect/DynObs.h"
using namespace std;

class Cluster{
    double x,y;
    int count;
    double x_dot, y_dot;
    double laset_seen;
    
    Cluster();
};

class LaserDetect{
public: 
    LaserDetect(const ros::NodeHandle& nh);
    //~LaserDetect();
    void laserCallback(const sensor_msgs::LaserScanConstPtr & msg);
    void matchClusters(vector<Cluster> clusters, double t);
    void getClusters(vector<float> ranges, vector<float> angles);
    
    
private:
    ros::Publisher dynObs_pub;
    ros::Subscriber scan_sub;
    ros::NodeHandle nh_;
    vector<Cluster> clusters;
    double last_t; 
    string laser_link;
    bool usescan;
    
    
};



#endif