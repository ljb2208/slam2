#pragma once

#include "Util/SLImage.h"
#include "Feature.h"
#include <cv.h>
#include <ctype.h>
#include <iostream>

class Features
{
    public:
        Features();
        void matchFeatures(SLImage* image, SLImage* imageRight);
        
    private:
        std::vector<cv::Point2f> getFeaturesToTrack(SLImage* image);
        cv::Point2f matchFeature(SLImage* image, SLImage* imageTarget, cv::Point2f feature);
        cv::Mat getImagePatch(cv::Mat image, cv::Point2f point, int width, int height);
        int calculateSAD(cv::Mat feature, cv::Mat search, int xOffset, int yOffset);
        int featureIndex;
};