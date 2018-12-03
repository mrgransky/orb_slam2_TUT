#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Syntax: \n./mono_kitti [path_to_vocabulary] [path_to_settings] [path_to_sequence]\n" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // ---------------------------------- Main loop ----------------------------------//
    Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
	chrono::steady_clock::time_point 	t1 = chrono::steady_clock::now();
#else
	chrono::monotonic_clock::time_point 	t1 = chrono::monotonic_clock::now();
#endif

        // Pass images to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        chrono::steady_clock::time_point 	t2 = chrono::steady_clock::now();
#else
        chrono::monotonic_clock::time_point 	t2 = chrono::monotonic_clock::now();
#endif

        double ttrack = chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    // ---------------------------------- Main loop ----------------------------------//

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "\n\n---------------------------" << endl;
    cout << "median tracking time: " 	<< vTimesTrack[nImages/2] 	<< endl;
    cout << "mean tracking time: " 	<< totaltime/nImages 		<< endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KF_Traj_KITTI.txt");  
    SLAM.SaveTrajectoryKITTI("Traj_KITTI.txt");
  
    SLAM.CreatePCD("mono_KITTI.pcd");
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
