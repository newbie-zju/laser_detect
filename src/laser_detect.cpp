#include "laser_detect/laser_detect.h"
#include "obstacle_avoidance/Hokuyo.h"

/****TODO: sensor_msgs/LaserScan
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities 
 * ****/
//由于安装镜子反射舍弃的激光角度
#define angleHead  (130.0*M_PI)/180
#define angleEnd  (140.0*M_PI)/180
#define VISUALIZE_DYNAMIC_OBSTACLES 1 //柱子
#define DATA_SZ 5
LaserDetect::LaserDetect(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh),nh_param("~")
{
    usescan =false;
    hokuyo_data.ranges.resize(DATA_SZ);
    hokuyo_data.angles.resize(DATA_SZ);
    
    hokuyo_data.ranges[0] = 999;
    hokuyo_data.ranges[1] = 999;
    hokuyo_data.ranges[2] = 999;
    hokuyo_data.ranges[3] = 999;
    hokuyo_data.ranges[4] = 999;
	
    hokuyo_data.angles[0] = 999;
    hokuyo_data.angles[1] = 999;
    hokuyo_data.angles[2] = 999;
    hokuyo_data.angles[3] = 999;
    hokuyo_data.angles[4] = 999;
    hokuyo_data.number = 0;
    
    nh_param.param<string>("laser_link_", laser_link, std::string("laser"));
    //最小探测距离
    nh_param.param<double>("minDist_",minDist,0.5);
    //最大探测距离
    nh_param.param<double>("maxDist_",maxDist,5.0);
    //探测距离差距
    nh_param.param<double>("clusterThreshold_",clusterThreshold,1.8); 
    //min number of points on obstacles
    nh_param.param<int>("clusterNMin_",clusterNMin,4);
    //max number of points on obstacles
    nh_param.param<int>("clusterNMax_",clusterNMax,50);
    //TODO:障碍物实际直径0.12m
    //障碍物直径下限(m)
    nh_param.param<double>("minWidth_",minWidth,0.05);
    //障碍物直径上限(m)
    nh_param.param<double>("maxWidth_", maxWidth,0.2);
    
    dynObs_pub = nh_.advertise<laser_detect::DynObs>("dynamic_obstacles",1);
    
    hokuyo_pub = nh_.advertise<obstacle_avoidance::Hokuyo>("/hokuyo/pillar_data",1);
#if VISUALIZE_DYNAMIC_OBSTACLES 
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker",1);
#endif
    
    scan_sub = nh_.subscribe("/scan",1, &LaserDetect::laserCallback,this);
}



void LaserDetect::getClusters(vector< double > ranges, vector< double > angles)
{
//     float minDist = 0.5; //最小探测距离
//     float maxDist = 2; //最大探测距离
//     
//     float clusterThreshold = 0.4; //探测距离差距
//     
//     unsigned int clusterNMin = 4; //min number of points on obstacles
//     unsigned int clusterNMax = 10;//max number of points on obstacles
//     //TODO:障碍物实际直径0.12m
//     float minWidth = 0.08; //障碍物直径下限(m)
//     float maxWidth = 0.15; //障碍物直径上限(m)
    
    
    //geometry_msgs::PointStamped msg_out;
    //geometry_msgs::PointStamped msg_in;
    int count=0;
    int order=1;
    geometry_msgs::PointStamped dynObs_laser;
    geometry_msgs::Point dynObs_ground;
    laser_detect::DynObs dynObs_points;
    Vector2d DynObs_data;
    vector <geometry_msgs::PointStamped> dynObs_temp_points;
    vector <Vector2d> DynObs_data_;
    
    for(unsigned int i=0; i<ranges.size(); i++)
    {
	unsigned int j;
	for (j=i; j < ranges.size()-1; j++)
	{
	    if (fabs(ranges[j+1]-ranges[j])>clusterThreshold)
		break;
	}
	//cout << "clusterThreshold:" << clusterThreshold << endl;
	//cout << "j-i:" << j-i << endl;
	//满足打到障碍物上的点数在设定范围，且这两点不是激光首尾点
	if((j-i)>=clusterNMin && (j-i) <= clusterNMax && i!=0 && j!=ranges.size()-1)
	{
	    int upperMid = ceil((float)(j+i)/2.); //正舍入
	    int lowerMid = (j+i)/2; //负舍入
	    //cout << "i j:" << i << " " << j << endl;
	    float dist = (ranges[upperMid]+ranges[lowerMid])/2;//该距离为障碍物到激光的平均距离
	    //cout << "dist:" << dist << endl;
	    //float width = dist*(angles[j]-angles[i]);//l=r*theta
	     
	    double width = dist*tan(angles[j]-angles[i]);
	    //double width2 = dist*abs(j-i)*0.00436332309619;
	    //cout << "width: " << width << endl;
	    //cout << "width2: " << width2 << endl;
	    //cout << "minWidth  maxWidth" << minWidth << " " << maxWidth << endl;
	    //cout << "minDist" << minDist << endl;
	    //满足障碍物直径在设定范围，且障碍物到激光的距离满足设定范围
	    if(width <= maxWidth && width >= minWidth && dist <= maxDist && dist >= minDist)
	    {
		//激光坐标系下的点
		float theta = (angles[j]+angles[i])/2;
		DynObs_data[0] = dist;
		DynObs_data[1] = theta;
		DynObs_data_.push_back(DynObs_data);
		//
		dynObs_laser.point.x = dist*cos(theta);
		dynObs_laser.point.y = dist*sin(theta);
		dynObs_laser.point.z = 0;
		dynObs_laser.header.frame_id = order;
		dynObs_laser.header.stamp = ros::Time(0);
		order ++;
		
		dynObs_temp_points.push_back(dynObs_laser);
		count ++;
		//dynObs_points.point.push_back(dynObs_laser);
		//dynObs_points.point_count = count;
		
	    }
	}
	i=j;
    }
    
    //cout << "count:" << count << endl;
    
    if (count == 0)
    {
	for(int i=count; i<5; i++)
	{
	    hokuyo_data.ranges[i] = 999;
	    hokuyo_data.angles[i] = 999;
	}
	DynObs_data_.clear();
	hokuyo_pub.publish(hokuyo_data);
    }
	
    else if(count > 5)
    {
	ROS_ERROR("Dynamic obstacles detected error! chose closest 5 dynobs!");
	
	Vector2d temp;
	for (int i = 0; i < (count-1); i++)
	{
	    //cout << "count:" << count << endl;
	    for (int j=i+1; j < count; j++)
	    {
		double dist_i = DynObs_data_[i][0];
		double dist_j = DynObs_data_[j][0];
		//cout << "dist_i:" << dist_i << endl;
		//cout << "dist_j:" << dist_j << endl;
		if (dist_j < dist_i)
		{
		    temp = DynObs_data_[i];
		    DynObs_data_[i] = DynObs_data_[j];
		    DynObs_data_[j] = temp;
		    //cout << "temp:" << temp << endl;
		}
	    }
	}
	
	for(int i=0; i<5; i++)
	{
	    hokuyo_data.ranges[i] = DynObs_data_[i][0];
	    hokuyo_data.angles[i] = DynObs_data_[i][1];
	    hokuyo_data.number = count;
	}
	DynObs_data_.clear();
	hokuyo_pub.publish(hokuyo_data);
	
    }  
    else
    {
	//cout << "count1:" << count << endl;
	//cout << "DynObs_data_origin:" << DynObs_data_[0][0] << " "<< DynObs_data_[1][0] << endl;
	//exchangeSort_(DynObs_data_,count);
	Vector2d temp;
	for (int i = 0; i < (count-1); i++)
	{
	    //cout << "count:" << count << endl;
	    for (int j=i+1; j < count; j++)
	    {
		double dist_i = DynObs_data_[i][0];
		double dist_j = DynObs_data_[j][0];
		//cout << "dist_i:" << dist_i << endl;
		//cout << "dist_j:" << dist_j << endl;
		if (dist_j < dist_i)
		{
		    temp = DynObs_data_[i];
		    DynObs_data_[i] = DynObs_data_[j];
		    DynObs_data_[j] = temp;
		    //cout << "temp:" << temp << endl;
		}
	    }
	}
	//cout << "DynObs_data_sort:" << DynObs_data_[0][0] << " "<< DynObs_data_[1][0] << endl;
	for(int i=0; i<count; i++)
	{
	    hokuyo_data.ranges[i] = DynObs_data_[i][0];
	    hokuyo_data.angles[i] = DynObs_data_[i][1];
	    hokuyo_data.number = count;
	}
	for(int i=count; i<5; i++)
	{
	    hokuyo_data.ranges[i] = 999;
	    hokuyo_data.angles[i] = 999;
	}
	DynObs_data_.clear();
	hokuyo_pub.publish(hokuyo_data);
    }    

    
    exchangeSort(dynObs_temp_points, count);
    
    for(int i=0; i< count; i++)
    {
	dynObs_points.point.push_back(dynObs_temp_points[i]);
	dynObs_points.point_count = count;
    }
   
    dynObs_points.header.frame_id = laser_link;
    dynObs_points.header.stamp = ros::Time(0);
    dynObs_pub.publish(dynObs_points);
    

#if VISUALIZE_DYNAMIC_OBSTACLES
    int dynObs_idx = 0;
    visualization_msgs::MarkerArray ma;
    for (unsigned int i=0; i<dynObs_points.point_count; i++)
    {
	geometry_msgs::Point test;
	geometry_msgs::Pose pose;
	pose.position.x = dynObs_points.point[i].point.x;
	pose.position.y = dynObs_points.point[i].point.y;
	pose.position.z = 0;
	
	visualization_msgs::Marker marker;
	marker.header = dynObs_points.header;
	marker.type = visualization_msgs::Marker::CYLINDER;//圆柱
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.ns = "dynamic_obstacles";
	marker.id = dynObs_idx;
	dynObs_idx ++;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(2);
	ma.markers.push_back(marker);
    }
#endif
    dynObs_points.point.clear();
    marker_pub.publish(ma);
    
}

void LaserDetect::exchangeSort(vector< geometry_msgs::PointStamped > dynObs_points_, int count_)
{
    geometry_msgs::PointStamped temp;
    for (int i=0; i<count_-1; i++)
    {
	for(int j=i+1; j<count_; j++)
	{
	    double dist_i = dynObs_points_[i].point.x * dynObs_points_[i].point.x + dynObs_points_[i].point.y * dynObs_points_[i].point.y;
	    
	    double dist_j = dynObs_points_[j].point.x * dynObs_points_[j].point.x + dynObs_points_[j].point.y * dynObs_points_[j].point.y;
	    
	    if(dist_j < dist_i)
	    {
		temp=dynObs_points_[i];
		dynObs_points_[i] = dynObs_points_[j];
		dynObs_points_[j] = temp;
	    }
	}
    }
}

void LaserDetect::exchangeSort_(vector<Eigen::Vector2d> dynObs_data_, int count_)
{
    Vector2d temp;
    for (int i = 0; i < (count_-1); i++)
    {
	for (int j=i+1; j < count_; j++)
	{
	    double dist_i = dynObs_data_[i][0];
	    double dist_j = dynObs_data_[j][0];
	    if (abs(dist_j - dist_i) < 0)
	    {
		temp = dynObs_data_[i];
		dynObs_data_[i] = dynObs_data_[j];
		dynObs_data_[j] = temp;
		cout << "temp:" << temp << endl;
	    }
	}
    }
}


//TODO: deal with laser data
void LaserDetect::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    vector<double> angles;
    vector<double> ranges;
    
    int num_rays = msg->ranges.size();
    //cout << "num_rays:" << num_rays << endl; 
    
    if (num_rays > 0)
	usescan =true;
    
    angles.reserve(num_rays);
    ranges.reserve(num_rays);
    
    float ang = msg-> angle_min; //min angle in hokuyo laser_detect_node
    /*********get the laser ranges and angles*********/
    for (int i = 0; i< num_rays; i++)
    {
	ranges.push_back(msg->ranges[i]);
	angles.push_back(ang);
	ang += msg->angle_increment; //angle distance between measurements[rad]
	
    }
/**********************************
        *hokuyo 
	*     -mirror-  
	* 0    1   2    3
	* \    |   |    /
	*  \   |   |   /
	*   \  |   |  /
	*    \ |   | /
	*     \|   |/ 
	*    --laser--
	* 0: msg->angle_min
	* 1: angleHead
	* 2: angleEnd
	* 3: msg->angle_max
 ********************************/
/*
    int num_0_1 = int((angleHead-msg->angle_min)/msg->angle_increment);
    int num_0_2 = int((angleEnd-msg->angle_min)/msg->angle_increment);
    int num_2_3 = int((msg->angle_max-angleEnd)/msg->angle_increment);
    int num_1_2 = int((angleHead-angleHead)/msg->angle_increment);
    int num_0_3 = int((msg->angle_max-msg->angle_min)/msg->angle_increment);
    //TODO:num_0_3 should be equal to num_rays
    ROS_ERROR_STREAM("use range num, num_rays:"<< num_rays <<". use angle num, num_0_3:"<<num_0_3);
    
    
    for(unsigned int i=0; i<num_0_1; i++)
    {
	ranges.push_back(msg->ranges[i]);
	angles.push_back(ang);
	ang += msg->angle_increment;
    }
    
    ang -= msg->angle_increment;
    ang += angleEnd-angleHead;
   
    for(unsigned int j=num_0_2; j<num_0_3; j++)
    {
	ranges.push_back(msg->ranges[j]);
	angles.push_back(ang);
	ang +=msg->angle_increment;
    }
*/  
    double t = msg->header.stamp.toSec();//(double)sec+1e-9*(double) nsec
    
    if(usescan)
    {
	getClusters(ranges,angles);
	usescan = false;
    }
    else
    {
	ROS_WARN("No laser scan data output...");
    }
    
//     ranges.clear();
//     angles.clear();
    
    
}

