#ifndef _LASER_DETECT_H_
#define _LASER_DETECT_H_

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <deque>
#include <stdio.h>
//#include "iarc_tf/ned_world_dynamic.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

#include "laser_detect/DynObs.h"
#include "obstacle_avoidance/Hokuyo.h"

#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

class Cluster{
    double x,y;
    int count;
    double x_dot, y_dot;
    double laset_seen;
    
    Cluster();
};

class LaserDetect{
public: 
    LaserDetect(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    //~LaserDetect();
    void laserCallback(const sensor_msgs::LaserScanConstPtr & msg);
    void matchClusters(vector<Cluster> clusters, double t);
    void getClusters(vector<double> ranges, vector<double> angles);
    void exchangeSort(vector<geometry_msgs::PointStamped> dynObs_points_, int count_);
    void exchangeSort(vector<Eigen::Vector2d> dynObs_data_, int count_);
    
    
private:
    ros::Publisher dynObs_pub;
    ros::Publisher marker_pub;
    ros::Publisher hokuyo_pub;
    ros::Subscriber scan_sub;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_param;
    vector<Cluster> clusters;
    double last_t; 
    string laser_link;
    bool usescan;
    double minDist; //最小探测距离
    double maxDist; //最大探测距离
    double sta_yaw;
    double clusterThreshold; //探测距离差距
    
    int clusterNMin ; //min number of points on obstacles
    int clusterNMax ;//max number of points on obstacles
    //TODO:障碍物实际直径0.12m
    double minWidth; //障碍物直径下限(m)
    double maxWidth; //障碍物直径上限(m)
    obstacle_avoidance::Hokuyo hokuyo_data;
    
};



#endif