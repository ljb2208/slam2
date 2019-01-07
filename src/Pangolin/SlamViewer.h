#pragma once

#include <pangolin/pangolin.h>
#include "boost/thread.hpp"
#include <map>
#include <deque>
#include <cv.h>

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

        cv::Mat internalVideoImg;
        cv::Mat internalVideoImgRight;


};