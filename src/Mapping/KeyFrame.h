#pragma once

#include "Odometry/Matcher.h"
#include "Odometry/Matches.h"
#include <vector>
#include "Util/SLImage.h"
//#include <Eigen/Core>
//#include "Util/NumType.h"

class KeyFrame
{
    public:
        KeyFrame();

        std::vector<Matches::p_match> p_matched;
        Matrix pose;
        int32_t index;
        bool temporary;
        SLImage* image;
        SLImage* imageRight;

        void generateDepthInfo();
        //SE3 getSE3();

    private:

        

        

        


};