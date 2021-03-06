#include <iostream>
#include <thread>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pangolin/pangolin.h>

#include "Converter.h"
#include "System.h"
#include "IMU/configParam.h"


#ifdef HAVE_OPENNI
#undef HAVE_OPENNI
#endif
#ifdef HAVE_OPENNI2
#undef HAVE_OPENNI2
#endif

using namespace std;
using namespace cv;
using namespace Eigen;


namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, const bool useIMU):
		mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), 
		mbReset(false), mbActivateLocalizationMode(false), 
		mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << "\n\n******************* ORB-SLAM2 ******************* " << endl;
    cout << "\n\nInput sensor: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

	// ------------------------ Visual Inertial added! ------------------------//
    //Check settings file
    FileStorage fsSettings(strSettingsFile.c_str(), FileStorage::READ);

    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    } 
    /*else 
    {
       // reading IMU configuration file:
	ConfigParam config(strSettingsFile);
    }*/

	// ------------------------ Visual Inertial added! ------------------------

    //Load ORB Vocabulary
    cout << "\nLoading ORB Vocabulary..." << endl;

    mpVocabulary 		= new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

	mpKeyFrameDatabase 	= new KeyFrameDatabase(*mpVocabulary);

	mpMap 			= new Map();
	mpFrameDrawer 		= new FrameDrawer(mpMap);
	mpMapDrawer 		= new MapDrawer(mpMap, strSettingsFile);

	mpTracker 		= new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, 
						mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

	mpLocalMapper 		= new LocalMapping(mpMap, mSensor==MONOCULAR);
	mptLocalMapping 	= new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
	mpLoopCloser 		= new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
	mptLoopClosing 		= new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

	if(bUseViewer)
	{
		mpViewer 	= new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker, strSettingsFile);
		mptViewer 	= new thread(&Viewer::Run, mpViewer);
		mpTracker->SetViewer(mpViewer);
	}

	mpTracker->SetLocalMapper(mpLocalMapper);
	mpTracker->SetLoopClosing(mpLoopCloser);

	mpLocalMapper->SetTracker(mpTracker);
	mpLocalMapper->SetLoopCloser(mpLoopCloser);

	mpLoopCloser->SetTracker(mpTracker);
	mpLoopCloser->SetLocalMapper(mpLocalMapper);

	// modification required!!!
	if(useIMU)
	{}
	/*if(ConfigParam::GetRealTimeFlag())
	{
		//Thread for VINS initialization
		mptLocalMappingVIOInit = new thread(&ORB_SLAM2::LocalMapping::VINSInitThread, mpLocalMapper);
	}*/
}

Mat System::TrackStereo(const Mat &imLeft, const Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Mat System::TrackRGBD(const Mat &im, const Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Mat System::TrackMonocular(const Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);

    mTrackingState 	= mpTracker->mState;
    mTrackedMapPoints 	= mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}


// ------------------------------Visual Inerial Added!------------------------------------- //
Mat System::TrackMonoVI(const Mat &im, const vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageMonoVI(im,vimu,timestamp);
}


bool System::bLocalMapAcceptKF()
{
    return (mpLocalMapper->AcceptKeyFrames() && !mpLocalMapper->isStopped());
    //return mpLocalMapper->ForsyncCheckNewKeyFrames();
}


void System::SaveKeyFrameTrajectoryNavState(const string &filename)
{
    cout << endl << "Saving keyframe NavState to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Eigen::Vector3d P = pKF->GetNavState().Get_P();
        Eigen::Vector3d V = pKF->GetNavState().Get_V();
        Eigen::Quaterniond q = pKF->GetNavState().Get_R().unit_quaternion();
        Eigen::Vector3d bg = pKF->GetNavState().Get_BiasGyr();
        Eigen::Vector3d ba = pKF->GetNavState().Get_BiasAcc();
        Eigen::Vector3d dbg = pKF->GetNavState().Get_dBias_Gyr();
        Eigen::Vector3d dba = pKF->GetNavState().Get_dBias_Acc();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " ";
        f << P(0) << " " << P(1) << " " << P(2) << " ";
        f << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        f << V(0) << " " << V(1) << " " << V(2) << " ";
        f << bg(0)+dbg(0) << " " << bg(1)+dbg(1) << " " << bg(2)+dbg(2) << " ";
        f << ba(0)+dba(0) << " " << ba(1)+dba(1) << " " << ba(2)+dba(2) << " ";
//        f << bg(0) << " " << bg(1) << " " << bg(2) << " ";
//        f << ba(0) << " " << ba(1) << " " << ba(2) << " ";
//        f << dbg(0) << " " << dbg(1) << " " << dbg(2) << " ";
//        f << dba(0) << " " << dba(1) << " " << dba(2) << " ";
        f << endl;
    }

    f.close();
    cout << endl << "NavState trajectory saved!" << endl;
}

// ------------------------------Visual Inerial Added!------------------------------------- //


void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
	{
		cout << "stuck in mpviewer..." << endl;
		usleep(5000);
	}
    }
    cout << "\nmpviewer deactivated!\n" << endl;

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
	cout << "stuck here ..." << endl;
        usleep(5000);
    }
    cout << "\nno while loop!!\n" << endl;

/*    if(mpViewer)
    {
	cout << "mpViewer still active!!" << endl;
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }*/
    cout << "end!!!" << endl;
}

void System::CreatePCD(const string &filename)
{
	vector<MapPoint*> vMPs = mpMap->GetAllMapPoints();
	cout << "size = " << vMPs.size() << endl;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	for (size_t i = 0; i < vMPs.size(); ++i)
	{
		pcl::PointXYZ p;

		MapPoint* pMP = vMPs[i];

        	if(pMP->isBad())
            		continue;

        	Mat MPPositions = pMP->GetWorldPos();
        	
		p.x = MPPositions.at<float>(0);
		p.y = MPPositions.at<float>(1);
		p.z = MPPositions.at<float>(2);

		cloud.push_back(p);
	}
	pcl::io::savePCDFileASCII(filename, cloud);


	/*if(mSensor==MONOCULAR)
		pcl::io::savePCDFileASCII("mono_cloud.pcd", cloud);
  	else if(mSensor==STEREO)
        	pcl::io::savePCDFileASCII("Stereo_cloud.pcd", cloud);
    	else if(mSensor==RGBD)
		pcl::io::savePCDFileASCII("RGBD_cloud.pcd", cloud);*/
	
	cout <<"\n[" << filename <<"] saved!"<< endl;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }*/

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).

    list<ORB_SLAM2::KeyFrame*>	::iterator lRit 		= mpTracker->mlpReferences.begin();
    list<double>		::iterator lT 			= mpTracker->mlFrameTimes.begin();
    list<bool>			::iterator lbL 			= mpTracker->mlbLost.begin();

    for(list<Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),lend=mpTracker->mlRelativeFramePoses.end();
	lit!=lend;
	lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Mat Trw = Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        Mat Tcw = (*lit)*Trw;
        Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f 	<< setprecision(6) << *lT 
		<< " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) 
		<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] 
		<< endl;
    }

    f.close();
    cout << "\n\nTrajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }*/

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).

    list<ORB_SLAM2::KeyFrame*>	::iterator lRit 	= mpTracker->mlpReferences.begin();
    list<double>		::iterator lT 		= mpTracker->mlFrameTimes.begin();


    for(list<Mat>::iterator lit	= mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); 
	lit != lend; 
	lit++, lRit++, lT++)
    {
        KeyFrame* pKF = *lRit;

        Mat Trw = Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        Mat Tcw = (*lit)*Trw;
        Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f 	<< setprecision(9) 	
		<< Rwc.at<float>(0,0) 	<< " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " " << twc.at<float>(0) << " "
		<< Rwc.at<float>(1,0) 	<< " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " " << twc.at<float>(1) << " " 
		<< Rwc.at<float>(2,0) 	<< " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " " << twc.at<float>(2) << endl;
    }

    f.close();
    cout << "\ntrajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << "\n\nSaving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Mat 		R = pKF->GetRotation().t();
        Mat 		t = pKF->GetCameraCenter();

	vector<float> 	q = Converter::toQuaternion(R);
        
	f 	<< setprecision(6) << pKF->mTimeStamp 								// timestamp
		<< setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) 	// xyz position
		<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] 					// quaternion
		<< endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}


vector<cv::Mat> System::getCurrentTrajectory()
{
	vector<cv::Mat> currentTraj;
	
	vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    	cv::Mat Two = vpKFs[0]->GetPoseInverse();

   	list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
	list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
	
	for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), 
		lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
	{
		ORB_SLAM2::KeyFrame* pKF = *lRit;

		cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

		while(pKF->isBad())
		{
			Trw = Trw*pKF->mTcp;
			pKF = pKF->GetParent();
		}

		//Trw = Trw*pKF->GetPose()*Two;  // keep the first frame on the origin
		Trw = Trw*pKF->GetPose();

		Mat Tcw 	= (*lit)*Trw;
		Mat Rwc 	= Tcw.rowRange(0,3).colRange(0,3).t();
		Mat twc 	= -Rwc*Tcw.rowRange(0,3).col(3);

		currentTraj.push_back(Tcw.inv());
	}
	return currentTraj;
}
int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
