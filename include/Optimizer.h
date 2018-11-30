
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

//#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{




// ------------------------------Visual Inerial Added!------------------------------------- //

public:
    void static LocalBAPRVIDP(	KeyFrame *pKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, 
				Map* pMap, cv::Mat& gw, LocalMapping* pLM = NULL);


    void static GlobalBundleAdjustmentNavStatePRV(	Map* pMap, const cv::Mat& gw, int nIterations, 
							bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust);


    void static LocalBundleAdjustmentNavStatePRV(	KeyFrame *pKF, const std::list<KeyFrame*> &lLocalKeyFrames, 
							bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM = NULL);

    void static GlobalBundleAdjustmentNavState(		Map* pMap, const cv::Mat& gw, int nIterations, bool* pbStopFlag, 
							const unsigned long nLoopKF, const bool bRobust);

    int static PoseOptimization(Frame *pFrame, KeyFrame* pLastKF, const IMUPreintegrator& imupreint, 
				const cv::Mat& gw, const bool& bComputeMarg=false);

    int static PoseOptimization(Frame *pFrame, Frame* pLastFrame, const IMUPreintegrator& imupreint, 
				const cv::Mat& gw, const bool& bComputeMarg=false);

    void static LocalBundleAdjustmentNavState(	KeyFrame *pKF, const std::list<KeyFrame*> &lLocalKeyFrames, 
						bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM = NULL);

    Vector3d static OptimizeInitialGyroBias(const std::list<KeyFrame*> &lLocalKeyFrames);

    Vector3d static OptimizeInitialGyroBias(const std::vector<KeyFrame*> &vLocalKeyFrames);

    Vector3d static OptimizeInitialGyroBias(const std::vector<Frame> &vFrames);

    Vector3d static OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt);

    void static LocalBundleAdjustment(KeyFrame *pKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, Map* pMap, LocalMapping* pLM=NULL);

// ------------------------------Visual Inerial Added!------------------------------------- //






public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);

    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);


// ------------------------------Visual Inerial Added!------------------------------------- //
    //void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, LocalMapping* pLM=NULL);
// ------------------------------Visual Inerial Added!------------------------------------- //


    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
