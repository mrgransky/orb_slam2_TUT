#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

#include "IMU/imuData.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class MapDrawer;

class System
{
public:
	enum eSensor {MONOCULAR = 0, STEREO = 1, RGBD = 2};

	Map* getMap() 
	{
		return mpMap;
	}

	Tracking* getTracker() 
	{
		return mpTracker;
	}

	LocalMapping* getLocalMapping()
	{
		return mpLocalMapper;
	}

	LoopClosing* getLoopClosing()
	{
		return mpLoopCloser;
	}

	MapDrawer* getMapDrawer()
	{
		return mpMapDrawer;
	}

	System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

	cv::Mat TrackStereo	(const cv::Mat &imLeft, const cv::Mat &imRight, 		const double &timestamp);
	cv::Mat TrackRGBD	(const cv::Mat &im, 	const cv::Mat &depthmap, 		const double &timestamp);
	cv::Mat TrackMonocular	(const cv::Mat &im, 						const double &timestamp);

	// ------------------------------Visual Inerial Added!------------------------------------- //
	cv::Mat TrackMonoVI	(const cv::Mat &im, 	const std::vector<IMUData> &vimu, 	const double &timestamp);
	bool bLocalMapAcceptKF();
	void SaveKeyFrameTrajectoryNavState(const string& filename);
	// ------------------------------Visual Inerial Added!------------------------------------- //

	std::vector<cv::Mat> getCurrentTrajectory();

	void ActivateLocalizationMode();
	void DeactivateLocalizationMode();

	bool MapChanged();

	void Reset();

	void Shutdown();

	void SaveTrajectoryTUM		(const string &filename);
	void SaveTrajectoryKITTI	(const string &filename);
	void SaveKeyFrameTrajectoryTUM	(const string &filename);

	void CreatePCD(const string &filename);

	int GetTrackingState();

	std::vector<MapPoint*> GetTrackedMapPoints();
	std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

	eSensor 		mSensor;

	ORBVocabulary* 		mpVocabulary;
	KeyFrameDatabase* 	mpKeyFrameDatabase;
	Map* 			mpMap;
	Tracking* 		mpTracker;
	LocalMapping*		mpLocalMapper;
	LoopClosing* 		mpLoopCloser;
	Viewer* 		mpViewer;
	FrameDrawer* 		mpFrameDrawer;
	MapDrawer* 		mpMapDrawer;
	
	std::thread* 		mptLocalMapping;
	std::thread* 		mptLoopClosing;
	std::thread* 		mptViewer;
	std::thread* 		mptLocalMappingVIOInit;


    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
