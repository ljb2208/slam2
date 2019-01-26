#include "Mapping.h"
#include "boost/chrono.hpp"

Mapping::Mapping(SlamViewer* viewer)
{
    this->viewer = viewer;
    running = true;
}

void Mapping::addFrame(Matrix pose, SLImage* leftImage, SLImage* rightImage, std::vector<Matcher::p_match> p_matched)
{
    KeyFrame keyFrame;
    keyFrame.index = leftImage->index;
    keyFrame.pose = pose;
    keyFrame.temporary = false;
    keyFrame.p_matched = p_matched;
    keyFrame.image = new SLImage(leftImage->w, leftImage->h, leftImage->timestamp, keyFrame.index);
    keyFrame.imageRight = new SLImage(rightImage->w, rightImage->h, rightImage->timestamp, keyFrame.index);

    boost::unique_lock<boost::mutex> lk(keyFramesMutex);
    keyFramesQueue.push(keyFrame);
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

        printf("Mapping: get next keyframe\n");
        bool success = getNextKeyFrame(&keyFrame);
        
        if (success)
        {
            delete keyFrame.image;
            delete keyFrame.imageRight;
        }
        else
            boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
    }
}

void Mapping::close()
{
    running = false;
}
