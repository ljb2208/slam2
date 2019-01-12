#include "Odometry.h"

Odometry::Odometry(SlamViewer* viewer)
{
    this->viewer = viewer;

    features = new Features();
}

void Odometry::addStereoFrames(SLImage* image, SLImage* imageRight)
{
    features->matchFeatures(image, imageRight);
    viewer->pushLiveImageFrame(image->imageColor, imageRight->imageColor);
}