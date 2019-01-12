#include "Feature.h"

Feature::Feature(int id, int rank, cv::Point2f point)
{
    this->id = id;
    this->rank = rank;
    this->point = point;
    age = 0;
}

void Feature::incrementAge()
{
    age++;
}