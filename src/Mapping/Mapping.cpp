#include "Mapping.h"
#include "boost/chrono.hpp"

Mapping::Mapping(SlamViewer* viewer)
{
    this->viewer = viewer;
    running = true;
}

void Mapping::addFrame(Matrix pose, SLImage* leftImage, SLImage* rightImage, std::vector<Matcher::p_match> p_matched)
{
    KeyFrame* keyFrame = new KeyFrame();
    keyFrame->index = leftImage->index;
    keyFrame->pose = Matrix(pose);
    keyFrame->temporary = false;
    keyFrame->p_matched = p_matched;
    keyFrame->image = new SLImage(leftImage->w, leftImage->h, leftImage->timestamp, keyFrame->index);
    keyFrame->imageRight = new SLImage(rightImage->w, rightImage->h, rightImage->timestamp, keyFrame->index);

    boost::unique_lock<boost::mutex> lk(keyFramesMutex);
    keyFramesQueue.push(*keyFrame);
}

bool Mapping::getNextKeyFrame(KeyFrame* keyFrame)
{
    boost::unique_lock<boost::mutex> lk(keyFramesMutex);

    if (keyFramesQueue.empty())
        return false;

    *keyFrame = keyFramesQueue.front();
    keyFramesQueue.pop();    

    return true;
}

void Mapping::run()
{
    while (running)
    {                
        KeyFrame keyFrame;        
        bool success = getNextKeyFrame(&keyFrame);
        
        if (success)
        {
            delete keyFrame.image;
            delete keyFrame.imageRight;

            if (!keyFrames.empty())
            {
                float distance = getTranslationDistance(&keyFrame);
                float angle = getRotationAngle(&keyFrame);
                printf("Distance between KFs: %f Angle: %f\n", distance, angle);
            }

            keyFrames.push_back(keyFrame);

            int count = 0;
            for(unsigned int i=0;i<keyFrames.size();i++)
            {
                if (keyFrames[i].pose.val != 0)
                    count++;
            }            

            viewer->pushKeyFrame(keyFrame);
        }
        else
            boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
    }
}

void Mapping::close()
{
    running = false;
}

float Mapping::getTranslationDistance(KeyFrame* keyFrame)
{
    // get euclidian distance between this keyframe and last keyframe on stack
    KeyFrame pKeyFrame = keyFrames.back();

    float f1, f2, f3;

    f1 = keyFrame->pose.val[0][3] - pKeyFrame.pose.val[0][3];
    f2 = keyFrame->pose.val[1][3] - pKeyFrame.pose.val[1][3];
    f3 = keyFrame->pose.val[2][3] - pKeyFrame.pose.val[2][3];

    return fabs(sqrt(f1*f1 + f2*f2 + f3*f3));    
}

float Mapping::getRotationAngle(KeyFrame* keyFrame)
{
    // get rotation angle between this keyframe and last keyframe on stack
    KeyFrame pKeyFrame = keyFrames.back();

    Matrix rA = keyFrame->pose.getMat(0, 0, 2, 2);
    Matrix rB = pKeyFrame.pose.getMat(0, 0, 2, 2);

    Matrix rAT = rA.operator~();
    Matrix rAB = rAT.operator*(rB);

    // calculate trace
    float trace = rAB.val[0][0] + rAB.val[1][1] + rAB.val[2][2];

    if (trace > 3)
        trace = 3;

    float f1 = acos((trace - 1) / 2);
    float f2 = (f1 * 180) / M_PI;

    //printf("trace: %f, f1: %f f2:%f f5: %f f6: %f\n", trace, f1, f2, f5, f6);

    return f2;
}