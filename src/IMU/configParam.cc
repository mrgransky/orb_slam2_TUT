#include "IMU/configParam.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace ORB_SLAM2
{
double ConfigParam::_g = 9.810;

Matrix4d ConfigParam::_EigTbc = Matrix4d::Identity();
Matrix4d ConfigParam::_EigTcb = Matrix4d::Identity();

Mat ConfigParam::_MatTbc = Mat::eye(4,4,CV_32F);
Mat ConfigParam::_MatTcb = Mat::eye(4,4,CV_32F);

string ConfigParam::_tmpFilePath = "";


bool ConfigParam::_bAccMultiply9p8 = false;
bool ConfigParam::_bRealTime = true;


double ConfigParam::_nVINSInitTime = 15;
double ConfigParam::_ImageDelayToIMU = 0;
int ConfigParam::_LocalWindowSize = 10;


ConfigParam::ConfigParam(std::string configfile)
{
    FileStorage fSettings(configfile, FileStorage::READ);

    cout << "\n\nParameters: " << endl;

    _testDiscardTime = fSettings["test.DiscardTime"];
    _nVINSInitTime = fSettings["test.VINSInitTime"];

    std::cout<<"VINS initialize time: "<<_nVINSInitTime<<std::endl;
    std::cout<<"Discart time in test data: "<<_testDiscardTime<<std::endl;

    fSettings["test.InitVIOTmpPath"] >> _tmpFilePath;
    std::cout<<"save tmp file in "<<_tmpFilePath<<std::endl;

    fSettings["bagfile"] >> _bagfile;
    std::cout<<"open rosbag: "<<_bagfile<<std::endl;
    fSettings["imutopic"] >> _imuTopic;
    fSettings["imagetopic"] >> _imageTopic;
    std::cout<<"imu topic: "<<_imuTopic<<std::endl;
    std::cout<<"image topic: "<<_imageTopic<<std::endl;

    _LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];
    std::cout<<"local window size: "<<_LocalWindowSize<<std::endl;

    _ImageDelayToIMU = fSettings["Camera.delaytoimu"];
    std::cout<<"timestamp image delay to imu: "<<_ImageDelayToIMU<<std::endl;

    {
        cv::FileNode Tbc_ = fSettings["Camera.Tbc"];
        Eigen::Matrix<double,3,3> R;
        R << Tbc_[0], Tbc_[1], Tbc_[2],
                Tbc_[4], Tbc_[5], Tbc_[6],
                Tbc_[8], Tbc_[9], Tbc_[10];
        Eigen::Quaterniond qr(R);
        R = qr.normalized().toRotationMatrix();
        Eigen::Matrix<double,3,1> t( Tbc_[3], Tbc_[7], Tbc_[11]);
        _EigTbc = Eigen::Matrix4d::Identity();
        _EigTbc.block<3,3>(0,0) = R;
        _EigTbc.block<3,1>(0,3) = t;
        _MatTbc = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTbc.at<float>(i,j) = _EigTbc(i,j);

        _EigTcb = Eigen::Matrix4d::Identity();
        _EigTcb.block<3,3>(0,0) = R.transpose();
        _EigTcb.block<3,1>(0,3) = -R.transpose()*t;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTcb.at<float>(i,j) = _EigTcb(i,j);

        // Tbc_[0], Tbc_[1], Tbc_[2], Tbc_[3], Tbc_[4], Tbc_[5], Tbc_[6], Tbc_[7], Tbc_[8], Tbc_[9], Tbc_[10], Tbc_[11], Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];
        std::cout<<"Tbc inited:"<<std::endl<<_EigTbc<<std::endl<<_MatTbc<<std::endl;
        std::cout<<"Tcb inited:"<<std::endl<<_EigTcb<<std::endl<<_MatTcb<<std::endl;
        std::cout<<"Tbc*Tcb:"<<std::endl<<_EigTbc*_EigTcb<<std::endl<<_MatTbc*_MatTcb<<std::endl;
    }

    {
        int tmpBool = fSettings["IMU.multiplyG"];
        _bAccMultiply9p8 = (tmpBool != 0);
        std::cout<<"whether acc*9.8? 0/1: "<<_bAccMultiply9p8<<std::endl;
    }

    {
        int tmpBool = fSettings["test.RealTime"];
        _bRealTime = (tmpBool != 0);
        std::cout<<"whether run realtime? 0/1: "<<_bRealTime<<std::endl;
    }
}

std::string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

Eigen::Matrix4d ConfigParam::GetEigTbc()
{
    return _EigTbc;
}

cv::Mat ConfigParam::GetMatTbc()
{
    return _MatTbc.clone();
}

Eigen::Matrix4d ConfigParam::GetEigT_cb()
{
    return _EigTcb;
}

cv::Mat ConfigParam::GetMatT_cb()
{
    return _MatTcb.clone();
}

int ConfigParam::GetLocalWindowSize()
{
    return _LocalWindowSize;
}

double ConfigParam::GetImageDelayToIMU()
{
    return _ImageDelayToIMU;
}

bool ConfigParam::GetAccMultiply9p8()
{
    return _bAccMultiply9p8;
}

}
