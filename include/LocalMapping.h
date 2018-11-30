#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "IMU/configParam.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{



// ------------------------------Visual Inerial Added!------------------------------------- //

public:
 ConfigParam* mpParams;

    // KeyFrames in Local Window, for Local BA
    // Insert in ProcessNewKeyFrame()
    void AddToLocalWindow(KeyFrame* pKF);
    void DeleteBadInLocalWindow(void);

    void VINSInitThread(void);
    bool TryInitVIO(void);
    bool GetVINSInited(void);
    void SetVINSInited(bool flag);

    bool GetFirstVINSInited(void);
    void SetFirstVINSInited(bool flag);

    cv::Mat GetGravityVec(void);
    cv::Mat GetRwiInit(void);

    bool GetMapUpdateFlagForTracking();
    void SetMapUpdateFlagInTracking(bool bflag);
    KeyFrame* GetMapUpdateKF();

    const KeyFrame* GetCurrentKF(void) const {return mpCurrentKeyFrame;}

    std::mutex mMutexUpdatingInitPoses;
    bool GetUpdatingInitPoses(void);
    void SetUpdatingInitPoses(bool flag);

    std::mutex mMutexInitGBAFinish;
    bool mbInitGBAFinish;
    bool GetFlagInitGBAFinish() { unique_lock<mutex> lock(mMutexInitGBAFinish); return mbInitGBAFinish; }
    void SetFlagInitGBAFinish(bool flag) { unique_lock<mutex> lock(mMutexInitGBAFinish); mbInitGBAFinish = flag; }

protected:
    double mnStartTime;
    bool mbFirstTry;
    double mnVINSInitScale;
    cv::Mat mGravityVec; // gravity vector in world frame
    cv::Mat mRwiInit;

    std::mutex mMutexVINSInitFlag;
    bool mbVINSInited;

    std::mutex mMutexFirstVINSInitFlag;
    bool mbFirstVINSInited;

    unsigned int mnLocalWindowSize;
    std::list<KeyFrame*> mlLocalKeyFrames;

    std::mutex mMutexMapUpdateFlag;
    bool mbMapUpdateFlagForTracking;
    KeyFrame* mpMapUpdateKF;

    bool mbUpdatingInitPoses;

    std::mutex mMutexCopyInitKFs;
    bool mbCopyInitKFs;
    bool GetFlagCopyInitKFs() { unique_lock<mutex> lock(mMutexCopyInitKFs); return mbCopyInitKFs; }
void SetFlagCopyInitKFs(bool flag) { unique_lock<mutex> lock(mMutexCopyInitKFs); mbCopyInitKFs = flag; }

// ------------------------------Visual Inerial Added!------------------------------------- //





public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestFinish();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    bool SetNotStop(bool flag);
    void SetAcceptKeyFrames(bool flag);
    

    void InterruptBA();

    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
