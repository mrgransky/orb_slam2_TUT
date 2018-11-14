#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream> 

using namespace std;
using namespace cv;
using namespace ros;

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  
  init(argc, argv, "cameraPub");
  NodeHandle nh;



  //string camera_name = "logitech_B525";
  string camera_name = "Android";

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_cam_msg = it.advertise(camera_name+"/image_raw", 1);
  Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>(camera_name+"/camera_info", 1);

  const string vsa = "http://192.168.43.26:8080/video?x.mjpeg";
 
 // IP webcam
  VideoCapture cap(vsa);
  //VideoCapture cap(0);

  // Check video is open
  if (!cap.isOpened())
  {
	cerr<<"Could not open video!!"<<endl;
	nh.shutdown();
	return 1;
  }
  cout << "Camera: ON ------------>>>>>>>>>>>>" << endl;

  Mat frame;

  sensor_msgs::ImagePtr cam_msg;

  // logitech, camera_info
  //const string camurl = "file:///home/farid/WS_Farid/orb_slam2_TUT/settingFiles/logitech_B525.yaml";
  
  // Android, camera_info
  const string camurl = "file:///home/farid/WS_Farid/orb_slam2_TUT/settingFiles/Huawei_P10.yaml";
  

  camera_info_manager::CameraInfoManager caminfo(nh, camera_name, camurl);

  sensor_msgs::CameraInfo ci;

  Rate loop_rate(5);

  while (nh.ok()) {

    cap >> frame;

    if(!frame.empty())
    {
		cam_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

		ci.header.stamp = ros::Time::now(); //
		ci=caminfo.getCameraInfo();

		pub_cam_msg.publish(cam_msg);
		pub_cam_info.publish(ci);

      		//imshow("video stream",frame);
      		waitKey(1); // 30 ms */
    }
    else
    {
	cout << "EMPTY FRAME!" << endl;
    }

    spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}
