#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


#include <opencv2/core/eigen.hpp>

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

ORB_SLAM2::System* mySLAM;

void pubAllPointCloud	(ORB_SLAM2::System* inpSLAM);
void pubRefPointCloud	(ORB_SLAM2::System* inpSLAM);
void pubCameraPath	(ORB_SLAM2::System* inpSLAM);
void pubCameraPose	(ORB_SLAM2::System* inpSLAM);

void getTrans2Gr	(const cv::Mat &inpCamMatrix);

cv::Mat getCameraMatrix	(ORB_SLAM2::System* inpSLAM);

ros::Publisher pub_all_cloud;
ros::Publisher pub_ref_cloud;
ros::Publisher pub_cam;
ros::Publisher pub_path;

ros::Subscriber sub_img;
ros::Time current_time, last_time;

class REAL_TIME_MONOCULAR
{
private:
public:
	REAL_TIME_MONOCULAR()
	{

	}
	void imgCallBack(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono");
	ros::start();

	ros::NodeHandle nh;

	if(argc != 4)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 Mono [path_to_vocabulary] [path_to_settings] camera_name" << endl;
		ros::shutdown();
		return 1;
	}

	stringstream ss;
	string camName;
	ss << argv[3];
	ss >> camName;

	while(ros::ok())
	{

		mySLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
		REAL_TIME_MONOCULAR rt_mono;
		
		sub_img 	= nh.subscribe(camName+"/image_raw", 1, &REAL_TIME_MONOCULAR::imgCallBack,&rt_mono);

		pub_all_cloud 	= nh.advertise		<sensor_msgs::PointCloud2>	("all_point_cloud"		,1);
		pub_ref_cloud 	= nh.advertise		<sensor_msgs::PointCloud2>	("ref_point_cloud"		,1);
		pub_cam 	= nh.advertise		<geometry_msgs::PoseStamped>	(camName+ "pose"		,1);
		pub_path 	= nh.advertise		<nav_msgs::Path>		(camName+ "path"		,1);
		
		ros::spin();
	}
	// Save camera trajectory
	mySLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTraj_ROS.txt");    
	mySLAM->CreatePCD();

	cout << "\n\nprocess ended, saving corresponding point cloud...\n" << endl;

	return 0;
}

void REAL_TIME_MONOCULAR::imgCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mySLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    // transformation between fixed frame (GROUND) -> vehicle
    getTrans2Gr(getCameraMatrix(mySLAM));

    pubCameraPose(mySLAM);

    pubAllPointCloud(mySLAM);
    pubRefPointCloud(mySLAM);
    
    pubCameraPath(mySLAM);
}

void getTrans2Gr(const cv::Mat &inpCamMatrix)
{
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	geometry_msgs::TransformStamped trans;
	
	trans.header.stamp = ros::Time::now();
	trans.header.frame_id = "odom";
	trans.child_frame_id = "base_link";

	tf2::Quaternion q;
	q.setRPY(-3.14/2, 0, 0);

	trans.transform.translation.x = 0;
	trans.transform.translation.y = 0;
	trans.transform.translation.z = 1;

	trans.transform.rotation.x = q.x();
	trans.transform.rotation.y = q.y();
	trans.transform.rotation.z = q.z();
	trans.transform.rotation.w = q.w();

	static_broadcaster.sendTransform(trans);

}


void pubAllPointCloud(ORB_SLAM2::System* inpSLAM)
{	
	PointCloud<PointXYZRGBA> all_cloud;
	
	// GetAllMapPoints() -> vector contains all map points
	vector<ORB_SLAM2::MapPoint*> mpAllPoints = inpSLAM->getMap()->GetAllMapPoints();



	//vector<ORB_SLAM2::MapPoint*> mpAllPoints = inpSLAM->GetTrackedMapPoints();
	//int nKFs = inpSLAM->getMap()->KeyFramesInMap();
        //int nMPs = inpSLAM->getMap()->MapPointsInMap();

        //cout << "\n\nKFs: " << nKFs << ", MPs: " << nMPs << " , TrackedMapPoints size = " << mpAllPoints.size() << endl;

	for (size_t i = 0; i < mpAllPoints.size(); ++i)
	{
		PointXYZRGBA p;

		ORB_SLAM2::MapPoint* pMP = mpAllPoints[i];

        	if(pMP->isBad())
            		continue;

        	cv::Mat MPPositions = pMP->GetWorldPos();
        	
		p.x = MPPositions.at<float>(0);
		p.y = MPPositions.at<float>(1);
		p.z = MPPositions.at<float>(2);

		p.r = 255;
		p.g = 255;
		p.b = 255;
		p.a = 255;

		all_cloud.push_back(p);
	}

	sensor_msgs::PointCloud2 ros_all_cloud;
	toROSMsg(all_cloud, ros_all_cloud);

	ros_all_cloud.header.frame_id = "base_link";

	ros_all_cloud.header.stamp = ros::Time::now();
	pub_all_cloud.publish(ros_all_cloud);
}

void pubRefPointCloud(ORB_SLAM2::System* inpSLAM)
{	
	PointCloud<PointXYZRGBA> ref_cloud;
	// GetReferenceMapPoints() -> vector contains all map points
	
	vector<ORB_SLAM2::MapPoint*> mpRefPoints = inpSLAM->getMap()->GetReferenceMapPoints();

	for (size_t i = 0; i < mpRefPoints.size(); ++i)
	{
		PointXYZRGBA p;

		ORB_SLAM2::MapPoint* pMP = mpRefPoints[i];

        	if(pMP->isBad())
            		continue;

        	cv::Mat MPPositions = pMP->GetWorldPos();
        	
		p.x = MPPositions.at<float>(0);
		p.y = MPPositions.at<float>(1);
		p.z = MPPositions.at<float>(2);
		p.r = 255;
		p.g = 0;
		p.b = 0;
		p.a = 255;

		ref_cloud.push_back(p);
	}

	sensor_msgs::PointCloud2 ros_ref_cloud;
	toROSMsg(ref_cloud, ros_ref_cloud);

	ros_ref_cloud.header.frame_id = "base_link";

	ros_ref_cloud.header.stamp = ros::Time::now();
	pub_ref_cloud.publish(ros_ref_cloud);
}


cv::Mat getCameraMatrix(ORB_SLAM2::System* inpSLAM)
{
	cv::Mat camMatrix;
	camMatrix = inpSLAM->getMapDrawer()->mCameraPose;
	return (camMatrix);
}

void pubCameraPose(ORB_SLAM2::System* inpSLAM)
{
	Eigen::Matrix4f temp;
	Eigen::Matrix4f cam2Gr;
	geometry_msgs::PoseStamped cam_pose;

	cv::Mat inpCamMatrix;
	inpCamMatrix = getCameraMatrix(inpSLAM);

	if (!inpCamMatrix.empty())
	{
		cv2eigen(inpCamMatrix.inv(),temp);
		//cam2Gr = camTrans * temp;
		cam2Gr = temp;
		
		cam_pose.pose.position.x = cam2Gr(0,3);
		cam_pose.pose.position.y = cam2Gr(1,3);
		cam_pose.pose.position.z = cam2Gr(2,3);
	
		Eigen::Matrix3f Rwc = cam2Gr.block<3,3>(0,0);
		Eigen::Quaternionf q(Rwc);
		
		//cout << "\nQuaternion scalar = " << q.w() << "\n(r,p,y):\n"<< (180/3.14)*q.vec() << endl;
		cam_pose.pose.orientation.x = q.x();
		cam_pose.pose.orientation.y = q.y();
		cam_pose.pose.orientation.z = q.z();
		cam_pose.pose.orientation.w = q.w();
		
		cam_pose.header.frame_id = "base_link";
		
		cam_pose.header.stamp = ros::Time::now();  
	} else
	{
		cout << "\nno feature detected!\n" << endl;;
	} 
	pub_cam.publish(cam_pose);
}

void pubCameraPath(ORB_SLAM2::System* inpSLAM)
{
	nav_msgs::Path cam_path;
	
	Eigen::Matrix4f temp;
	Eigen::Matrix4f path2Gr;

	geometry_msgs::PoseStamped cam_pose;

	cv::Mat inpCamMatrix;
	inpCamMatrix = getCameraMatrix(inpSLAM);
	
	if (!inpCamMatrix.empty())
	{
		vector <cv::Mat> vec_traj;
		vec_traj = inpSLAM->getCurrentTrajectory();
		for(auto mt:vec_traj)
		{
			cv2eigen(mt,temp);
			//cam2Gr = camTrans * temp;
			path2Gr = temp;
		
			cam_pose.pose.position.x = path2Gr(0,3);
			cam_pose.pose.position.y = path2Gr(1,3);
			cam_pose.pose.position.z = path2Gr(2,3);
	
			Eigen::Matrix3f Rwc = path2Gr.block<3,3>(0,0);
			Eigen::Quaternionf q(Rwc);
		
			cam_pose.pose.orientation.x = q.x();
			cam_pose.pose.orientation.y = q.y();
			cam_pose.pose.orientation.z = q.z();
			cam_pose.pose.orientation.w = q.w();

			cam_path.poses.push_back(cam_pose);
		}
		
		cam_path.header.frame_id = "base_link";
		cam_path.header.stamp = ros::Time::now();
	} else
	{
		cout << "\nno feature detected!\n" << endl;;
	} 
	pub_path.publish(cam_path);
}
