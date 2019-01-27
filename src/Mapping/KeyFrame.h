#pragma once

#include "Odometry/Matcher.h"
#include <vector>
#include "Util/SLImage.h"

class KeyFrame
{
    public:
        KeyFrame();

        std::vector<Matcher::p_match> p_matched;
        Matrix pose;
        int32_t index;
        bool temporary;
        SLImage* image;
        SLImage* imageRight;

        float getTranslationData(float* data);
        
    private:

        

        


};