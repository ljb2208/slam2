#pragma once

#include <pangolin/pangolin.h>
#include "boost/thread.hpp"
#include <map>
#include <deque>
#include <cv.h>
#include "KeyFrameDisplay.h"

class SlamViewer{
    public:
        SlamViewer(int width, int height);
        ~SlamViewer();

        void run();
	    void close();
        void join();

        void pushLiveImageFrame(cv::Mat image, cv::Mat imageRight);
        void VideoSample(const std::string uri);

    private:
        int width, height;
        bool running;
        bool videoImageChanged;

        boost::thread runThread;
	    boost::mutex openImagesMutex;
        boost::mutex model3DMutex;

        cv::Mat internalVideoImg;
        cv::Mat internalVideoImgRight;

        std::vector<KeyFrameDisplay*> keyframes;
        KeyFrameDisplay* currentCam;

        // render settings
        bool settings_showKFCameras;
        bool settings_showCurrentCamera;
        bool settings_showTrajectory;
        bool settings_showFullTrajectory;
        bool settings_showActiveConstraints;
        bool settings_showAllConstraints;

       	float settings_scaledVarTH;
        float settings_absVarTH;
        int settings_pointCloudMode;
        float settings_minRelBS;
        int settings_sparsity;

};