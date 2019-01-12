#pragma once

#include <cv.h>

class SLImage
{
    public:
        SLImage();
        SLImage(int width, int height, double timestamp)
        {
            this->w = width;
            this->h = height;
            this->timestamp = timestamp;
        };
        cv::Mat image;
        cv::Mat imageColor;
        int w, h;
        double timestamp;
        float exposure_time;
};