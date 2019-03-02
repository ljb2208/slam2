#pragma once

#include "Pangolin/SlamViewer.h"
#include "Odometry/Matcher.h"
#include "Odometry/Matches.h"
#include <vector>
#include <queue>
#include "Util/SLImage.h"
#include "KeyFrame.h"
#include "boost/thread.hpp"

class Mapping
{
    public:
        Mapping(SlamViewer* viewer);

         // general parameters
        struct parameters {
            double  translation_threshold;
            double  rotation_threshold;
            double  search_radius;
            double  search_angle;
            int32_t keyframe_gap;

            parameters () {
                translation_threshold = 1.0;
                rotation_threshold = 0.1;
                search_radius = 10;
                search_angle = 15;
                keyframe_gap = 15;
            }
        };


        void addFrame(Matrix pose, SLImage* leftImage, SLImage* rightImage, Matches* matches);

        void run();
	    void close();

    private:
        SlamViewer* viewer;
        std::vector<KeyFrame> keyFrames;
        std::queue<KeyFrame> keyFramesQueue;

        boost::mutex keyFramesMutex;

        bool getLastKeyFrame(KeyFrame* keyFrame);
        bool getNextKeyFrameFromQueue(KeyFrame* keyFrame);
        float getTranslationDistance(KeyFrame* keyFrame, KeyFrame* keyFrame2);
        float getRotationAngle(KeyFrame* keyFrame, KeyFrame* keyFrame2);

        std::vector<KeyFrame> getPotentialLoopClosureKFs(KeyFrame* keyFrame);
        KeyFrame currentKeyFrame;

        // parameters
        parameters param;

        bool running;
};