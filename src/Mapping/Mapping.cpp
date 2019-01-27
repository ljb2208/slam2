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
