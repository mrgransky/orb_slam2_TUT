#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include"../../../include/System.h"

using namespace std;
using namespace pcl;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");

	if(argc != 3)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 node_publisher [path_to_vocabulary] [path_to_settings]" << endl;
		ros::shutdown();
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	
	ros::NodeHandle nh;
	ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	
	PointCloud<PointXYZ> cloud;
	
	vector<ORB_SLAM2::MapPoint*> mpPoints = SLAM.getMap()->GetAllMapPoints();
	//vector<ORB_SLAM2::MapPoint*> mpPoints = SLAM.GetTrackedMapPoints();
	
	for (size_t i = 0; i < mpPoints.size(); ++i)
	{
		PointXYZ p;

		ORB_SLAM2::MapPoint* pMP = mpPoints[i];

        	if(pMP->isBad())
            		continue;

        	cv::Mat MPPositions = pMP->GetWorldPos();
        	
		p.x = MPPositions.at<float>(0);
		p.y = MPPositions.at<float>(1);
		p.z = MPPositions.at<float>(2);

		cloud.push_back(p);
	}



	sensor_msgs::PointCloud2 ros_cloud;
	toROSMsg(cloud, ros_cloud);

	ros_cloud.header.frame_id = "point_ccloud";
	
	
	pub_cloud.publish(ros_cloud);
	ros::spin();



	return 0;
}
