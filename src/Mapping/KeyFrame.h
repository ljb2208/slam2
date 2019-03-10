#pragma once

#include "Odometry/Matrix.h"
#include "Odometry/Matcher.h"
#include "Odometry/Matches.h"
#include <vector>
#include "Util/NumType.h"
#include "Util/SLImage.h"
#include "Histogram.h"

class KeyFrame
{
    public:
        KeyFrame();
        KeyFrame(int32_t index, int32_t width, int32_t height, slam2::Matrix pose);
        ~KeyFrame();

        std::vector<Matches::p_match> p_matched;
        slam2::Matrix pose;
        int32_t index;
        int32_t width, height;
        bool temporary;

        // SLImage* image;
        // SLImage* imageRight;

        void generateDepthInfo();
        void generateHistogram(Matches* matches);
        Sophus::SE3 getSE3();
        void calculateAngleIncrements(KeyFrame prevKeyFrame);

        Histogram hist;

        float x, y, z;
        float x_inc, y_inc, z_inc;

    private:

        void calculateAngles();
        float angleDiff(float a1, float a2);
        bool closeEnough(const float& a, const float& b, const float& epsilon=std::numeric_limits<float>::epsilon());

        

        

        


};