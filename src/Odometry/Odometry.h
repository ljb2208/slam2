#pragma once

#include "Util/SLImage.h"
#include "Pangolin/SlamViewer.h"
#include "Features.h"

class Odometry
{
    public:
        Odometry(SlamViewer* viewer);
        void addStereoFrames(SLImage* image, SLImage* imageRight);

    private:
        SlamViewer* viewer;
        Features* features;

};