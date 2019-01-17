#pragma once

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <boost/thread.hpp>

#include "Util/Undistorter.h"
#include "Util/SLImage.h"

inline std::string type2str(int type)
{
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
        case CV_8U: r = "8U"; break;
        case CV_8S: r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        default: r="User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

class ImageReader{

public:
    ImageReader(bool displayImage, std::string paramFile);
    ~ImageReader();

    bool loadImage(std::string fileName);
    cv::Mat getImage();
    SLImage* getUndistortedImage(double timestamp);
    int getImageWidth();
    int getImageHeight();
    int getUDistImageWidth();
    int getUDistImageHeight();
    cv::Mat getCameraMatrix();
    float getBaseline();

private:
    bool displayImage;
    cv::Mat image;
    cv::Mat imageColor;
    Undistorter* udist;
};
