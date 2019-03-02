#pragma once

#include <pangolin/pangolin.h>
#include "boost/thread.hpp"
#include <map>
#include <deque>
#include <cv.h>
#include "KeyFrameDisplay.h"
#include "Mapping/KeyFrame.h"
#include "Util/Settings.h"

struct GraphConnection
{
	KeyFrameDisplay* from;
	KeyFrameDisplay* to;
	int fwdMarg, bwdMarg, fwdAct, bwdAct;
};


class SlamViewer{
    public:
        SlamViewer(int width, int height);
        ~SlamViewer();

        void run();
	    void close();
        void join();

        void pushKeyFrame(KeyFrame keyFrame);
        void pushDepthImageFrame(cv::Mat image);
        void pushLiveImageFrame(cv::Mat image, cv::Mat imageRight, int imageId);        

    private:
        int width, height;
        int currentImageId;
        bool running;
        bool videoImageChanged;

        boost::thread runThread;
	    boost::mutex openImagesMutex;
        boost::mutex model3DMutex;

        cv::Mat internalVideoImg;
        cv::Mat internalVideoImgRight;

        std::vector<KeyFrameDisplay*> keyFrames;
        KeyFrameDisplay* currentCam;
        std::vector<GraphConnection,Eigen::aligned_allocator<GraphConnection>> connections;
        std::vector<Vec3f,Eigen::aligned_allocator<Vec3f>> allFramePoses;

        // render settings
        bool settings_showKFCameras;
        bool settings_showCurrentCamera;
        //bool settings_showTrajectory;
        bool settings_showFullTrajectory;
        bool settings_showActiveConstraints;
        bool settings_showAllConstraints;

       	float settings_scaledVarTH;
        float settings_absVarTH;
        int settings_pointCloudMode;
        float settings_minRelBS;
        int settings_sparsity;

        void drawConstraints();

        std::vector<Matrix> groundTruth;

};