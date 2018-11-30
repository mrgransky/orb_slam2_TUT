

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include <Eigen/Dense>

//#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
//#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

#include "IMU/imuPreintegrator.h"
#include "IMU/navState.h"
namespace ORB_SLAM2
{

class Converter
{
public:


	// ------------------------------Visual Inerial Added!------------------------------------- //
	static void updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw);
	static cv::Mat toCvMatInverse(const cv::Mat &T12);
	// ------------------------------Visual Inerial Added!------------------------------------- //



    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
