#pragma once

#include "Odometry/Matcher.h"
#include "Odometry/Matches.h"
#include <vector>
#include "Util/SLImage.h"

class KeyFrame
{
    public:
        KeyFrame();
        KeyFrame(int32_t index, int32_t width, int32_t height);

        std::vector<Matches::p_match> p_matched;
        Matrix pose;
        int32_t index;
        int32_t width, height;
        bool temporary;
        //SLImage* image;
        //SLImage* imageRight;

        void generateDepthInfo();
        //Sophus::SE3 getSE3();

    private:

        

        

        


};