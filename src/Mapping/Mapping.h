#pragma once

#include "Pangolin/SlamViewer.h"
#include "Odometry/Matcher.h"
#include <vector>
#include <queue>
#include "Util/SLImage.h"
#include "KeyFrame.h"
#include "boost/thread.hpp"

class Mapping
{
    public:
        Mapping(SlamViewer* viewer);

        void addFrame(Matrix pose, SLImage* leftImage, SLImage* rightImage, std::vector<Matcher::p_match> p_matched);

        void run();
	    void close();

    private:
        SlamViewer* viewer;
        std::vector<KeyFrame> keyFrames;
        std::queue<KeyFrame> keyFramesQueue;

        boost::mutex keyFramesMutex;

        bool getNextKeyFrame(KeyFrame* keyFrame);
        KeyFrame currentKeyFrame;

        bool running;
};