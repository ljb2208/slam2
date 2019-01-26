#pragma once

#include <cv.h>

class SLImage
{
    public:
        SLImage();
        SLImage(int width, int height, double timestamp, int index)
        {
            this->w = width;
            this->h = height;
            this->timestamp = timestamp;
            this->index = index;
        };
        cv::Mat image;
        cv::Mat imageColor;
        int w, h, index; 
        double timestamp;
        float exposure_time;
};