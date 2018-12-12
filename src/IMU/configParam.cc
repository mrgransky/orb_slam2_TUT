#include "IMU/configParam.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace ORB_SLAM2
{


	Matrix4d ConfigParam::_EigTbc = Matrix4d::Identity();
	Matrix4d ConfigParam::_EigTcb = Matrix4d::Identity();

	Mat ConfigParam::_MatTbc = Mat::eye(4,4,CV_32F);
	Mat ConfigParam::_MatTcb = Mat::eye(4,4,CV_32F);

	string ConfigParam::_tmpFilePath = "";


	bool ConfigParam::_bAccMultiply9p8 	= false;
	bool ConfigParam::_bRealTime 		= true;


	double ConfigParam::_g 			= 9.810;
	double ConfigParam::_nVINSInitTime 	= 15;
	double ConfigParam::_ImageDelayToIMU 	= 0;
	int ConfigParam::_LocalWindowSize 	= 10;


ConfigParam::ConfigParam(string configfile)
{
	FileStorage fSettings(configfile, FileStorage::READ);

	cout << "\n\nParameters: " << endl;

	_testDiscardTime 	= fSettings["test.DiscardTime"];
	_nVINSInitTime 		= fSettings["test.VINSInitTime"];
	_ImageDelayToIMU 	= fSettings["Camera.delaytoimu"];
	_LocalWindowSize 	= fSettings["LocalMapping.LocalWindowSize"];

	cout << "VINS initialize time: "	<<_nVINSInitTime	<<endl;
	cout << "Discart time in test data: "	<<_testDiscardTime	<<endl;

	fSettings["test.InitVIOTmpPath"] 	>> _tmpFilePath;
	fSettings["bagfile"] 			>> _bagfile;
	fSettings["imutopic"] 			>> _imuTopic;
	fSettings["imagetopic"] 		>> _imageTopic;

	cout << "open rosbag: "			<<_bagfile		<<endl;
	cout << "imu topic: "			<<_imuTopic		<<endl;
	cout << "image topic: "			<<_imageTopic		<<endl;
	cout << "save tmp file in "		<<_tmpFilePath		<<endl;
	cout << "local window size: "		<<_LocalWindowSize	<<endl;
	cout << "Ts image delay 2 imu: "	<<_ImageDelayToIMU	<<endl;

    {
        FileNode Tbc_ 				= fSettings["Camera.Tbc"];

        Matrix<double,3,3> R;
	Matrix<double,3,1> t;

	t <<	Tbc_[3],
		Tbc_[7],
		Tbc_[11];

        R << 	Tbc_[0], Tbc_[1], Tbc_[2],
                Tbc_[4], Tbc_[5], Tbc_[6],
                Tbc_[8], Tbc_[9], Tbc_[10];

        Quaterniond qr(R);

        R = qr.normalized().toRotationMatrix();

        
	_EigTbc = Matrix4d::Identity();
        _EigTbc.block<3,3>(0,0) = R;
        _EigTbc.block<3,1>(0,3) = t;
        _MatTbc = Mat::eye(4,4,CV_32F);

	for(int i=0 ;i<4 ;i++)
	{
		for(int j=0; j<4; j++)
		{
			_MatTbc.at<float>(i,j) = _EigTbc(i,j);
		}

	}

        _EigTcb = Eigen::Matrix4d::Identity();
        _EigTcb.block<3,3>(0,0) = R.transpose();
        _EigTcb.block<3,1>(0,3) = -R.transpose()*t;

	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			_MatTcb.at<float>(i,j) = _EigTcb(i,j);
		}
	}

	// Tbc_[0], Tbc_[1], Tbc_[2], Tbc_[3], 
	// Tbc_[4], Tbc_[5], Tbc_[6], Tbc_[7], 
	// Tbc_[8], Tbc_[9], Tbc_[10], Tbc_[11], 
	// Tbc_[12], Tbc_[13], Tbc_[14], Tbc_[15];

	cout <<"Tbc inited:\n "	<< _EigTbc		<<"\n"<< _MatTbc		<<endl;
	cout <<"Tcb inited:\n"	<< _EigTcb		<<"\n"<< _MatTcb		<<endl;
	cout <<"Tbc*Tcb: \n"	<< _EigTbc*_EigTcb	<<"\n"<< _MatTbc*_MatTcb	<<endl;
    }



    {
        int tmpBool 		= fSettings["IMU.multiplyG"];
        _bAccMultiply9p8 = (tmpBool != 0);
        cout <<"whether acc*9.8? 0/1: " << _bAccMultiply9p8 << endl;
    }

    {
        int tmpBool 		= fSettings["test.RealTime"];
        _bRealTime = (tmpBool != 0);
        cout <<"whether run realtime? 0/1: "<< _bRealTime << endl;
    }
}

string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

Matrix4d ConfigParam::GetEigTbc()
{
    return _EigTbc;
}

Mat ConfigParam::GetMatTbc()
{
    return _MatTbc.clone();
}

Matrix4d ConfigParam::GetEigT_cb()
{
    return _EigTcb;
}

Mat ConfigParam::GetMatT_cb()
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
