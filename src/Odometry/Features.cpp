
#include "Features.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <ctype.h>
#include <iostream>

Features::Features()
{
    featureIndex = 0;
}

void Features::matchFeatures(SLImage* image, SLImage* imageRight)
{
    std::vector<cv::Point2f> corners = getFeaturesToTrack(image);
    std::vector<cv::Point2f> cornersRight = getFeaturesToTrack(imageRight);

    for (int i=0; i < corners.size(); i++)
    {
        cv::Point2f matchPoint = matchFeature(image, imageRight, corners[i]);
        cv::circle(imageRight->imageColor, matchPoint, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    }
}

std::vector<cv::Point2f> Features::getFeaturesToTrack(SLImage* image)
{
    cv::Size subPixWinSize(10, 10);
    cv::Size winSize(31, 31);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    
    std::vector<cv::Point2f> corners;

    cv::goodFeaturesToTrack(image->image, corners, 500, 0.01, 10.0, cv::Mat(), 3, false, 0.04);
    cv::cornerSubPix(image->image, corners, subPixWinSize, cv::Size(-1, -1), termcrit);

    for (int i=0; i < corners.size(); i++)
    {
        cv::circle(image->imageColor, corners[i], 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    }

    printf("Corners detected: %i\n", static_cast<int>(corners.size()));

    return corners;
}

cv::Point2f Features::matchFeature(SLImage* image, SLImage* imageTarget, cv::Point2f feature)
{
    //cv::Mat feature(4, 4, CV_8UC1);
    int searchSize = 20;
    int featureSize = 4;

    cv::Point2f matchFeature;
    cv::Point2f bestResult;

    cv::Mat origFeature = getImagePatch(image->image, feature, featureSize, featureSize);
    cv::Mat searchMat = getImagePatch(imageTarget->image, feature, searchSize, searchSize);

    printf("Got image patches\n");

    int sadMin = 255 * featureSize * featureSize; 

    for (int i=0; i < searchSize - featureSize; i++)
    {
        for (int y=0; y < searchSize - featureSize; y++)
        {
            int sad = calculateSAD(origFeature, searchMat, i, y);

            if (sad < sadMin)
            {
                sadMin = sad;
                bestResult.x = feature.x + static_cast<float>(i);
                bestResult.y = feature.y + (float)y;
            }
        }
    }

    printf("Min SAD: %i\n", sadMin);


    printf("feature point x: %f y: %f\n", feature.x, feature.y);

    return bestResult;
}

int Features::calculateSAD(cv::Mat feature, cv::Mat search, int xOffset, int yOffset)
{
    int sad = 0;

    for (int i=0; i < feature.rows; i++)
    {
        for (int y=0; y < feature.cols; y++)
        {
            sad += abs(feature.at<uchar>(i, y) - search.at<uchar>(i + xOffset, y + yOffset));
        }
    }

    return sad;
}

cv::Mat Features::getImagePatch(cv::Mat image, cv::Point2f point, int width, int height)
{
    int x = (int) point.x;
    int y = (int) point.y;

    int startx = x - width/2;
    int starty = y - height/2;

    int endx = x + width/2;
    int endy = y + height/2;

    if (startx < 0)
        startx = 0;
    
    if (starty < 0)
        starty = 0;

    if (endx > image.cols)
        endx = image.cols;
    
    if (endy > image.rows)
        endy = image.rows;
    
    cv::Rect range;
    range.x = startx;
    range.y = starty;
    range.width = endx - startx;
    range.height = endy - starty;
    cv::Mat roi(image, range);

    return roi;
}

void Features::computeFeatures()
{
    
}