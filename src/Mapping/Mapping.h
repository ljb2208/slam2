#pragma once

#include "Pangolin/SlamViewer.h"
#include "Odometry/Matcher.h"
#include "Odometry/Matches.h"
#include <vector>
#include <queue>
#include "Util/SLImage.h"
#include "KeyFrame.h"
#include "boost/thread.hpp"
#include "IO/DataSetReader.h"
#include <cv.h>
#include "G2ODriver.h"

class SADKeyFrame
{
    public:
        int sad;
        KeyFrame keyFrame;
};

class Mapping
{
    public:
        Mapping(SlamViewer* viewer, cv::Mat cameraMatrix, float baseLine, int imageHeight, int imageWidth);
        ~Mapping();
        // camera parameters (all are mandatory / need to be supplied)
        struct calibration {  
            double f;  // focal length (in pixels)
            double fy;
            double cu; // principal point (u-coordinate)
            double cv; // principal point (v-coordinate)
            calibration () {
            f  = 1;
            fy = 1;
            cu = 0;
            cv = 0;
            }
        };

        


         // general parameters
        struct parameters {
            double  translation_threshold;
            double  rotation_threshold;
            double  search_radius;
            double  search_angle;
            int32_t keyframe_gap;
            float   angle_change_threshold;
            int32_t max_keyframes_tocheck;
            double  baseline;
            int32_t height;
            int32_t width;
            int32_t inlier_threshold;
            calibration calib;          
           

            parameters () {
                translation_threshold = 1.0;
                rotation_threshold = 0.1;
                search_radius = 30;
                search_angle = 15;
                angle_change_threshold = 5.0;
                max_keyframes_tocheck = 5;
                baseline = 0.53;
                height = 640;
                width = 480;
                inlier_threshold = 150;
            }
        };

        
        void addFrame(slam2::Matrix pose, SLImage* leftImage, SLImage* rightImage, Matches* matches);

        void run();
	    void close();

        void setImageAttributes(ImageFolderReader* reader, std::string param);

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
        std::vector<SADKeyFrame> filterPotentialKFsBySAD(KeyFrame keyFrame, std::vector<KeyFrame> potentialKeyFrames);
        void matchKeyFrames(KeyFrame* keyFrame, std::vector<SADKeyFrame> kfsToMatch);
        KeyFrame currentKeyFrame;
        

        // parameters
        parameters param;

        // image params
        std::string camera_param;
        ImageFolderReader* reader;

        bool running;

        std::ofstream outputFile;
        int optimizationCount;
};