#pragma once
#include <cv.h>

class Feature
{
    public:
        Feature(int id, int rank, cv::Point2f point);
        void incrementAge();

    private:
        int age;
        int rank;
        int id;
        cv::Point2f point;
};