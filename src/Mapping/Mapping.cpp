#include "Mapping.h"
#include "boost/chrono.hpp"
#include <stdlib.h>

Mapping::Mapping(SlamViewer* viewer)
{
    this->viewer = viewer;
    running = true;
}

void Mapping::addFrame(slam2::Matrix pose, SLImage* leftImage, SLImage* rightImage, Matches* matches)
{
    KeyFrame* keyFrame = new KeyFrame(leftImage->index, leftImage->w, leftImage->h, pose);
    // keyFrame->index = leftImage->index;
    //keyFrame->pose = ;
    keyFrame->temporary = false;
    keyFrame->generateHistogram(matches);
    // keyFrame->image = leftImage;
    // keyFrame->imageRight = rightImage;
    
    // Need to fix
    keyFrame->p_matched = matches->copySelectedMatches();
    //printf("Add Frame: %i\n", keyFrame->index);
    //keyFrame->image = new SLImage(leftImage->w, leftImage->h, leftImage->timestamp, keyFrame->index);
    //keyFrame->imageRight = new SLImage(rightImage->w, rightImage->h, rightImage->timestamp, keyFrame->index);

    boost::unique_lock<boost::mutex> lk(keyFramesMutex);
    keyFramesQueue.push(*keyFrame);
}

bool Mapping::getLastKeyFrame(KeyFrame* keyFrame)
{
    if (keyFrames.size() == 0)
        return false;

    *keyFrame = keyFrames.back();

    return true;
}

bool Mapping::getNextKeyFrameFromQueue(KeyFrame* keyFrame)
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
        bool success = getNextKeyFrameFromQueue(&keyFrame);
        
        if (success)
        {
            //delete keyFrame.image;
            //delete keyFrame.imageRight;

            float distance = 0.0;
            float angle = 0.0;

            KeyFrame keyFrame2;

            if (getLastKeyFrame(&keyFrame2))
            {
                distance = getTranslationDistance(&keyFrame, &keyFrame2);
                angle = getRotationAngle(&keyFrame, &keyFrame2);

                // discard keyframe if not enough rotation or translation to previous keyframe
                if (distance < param.translation_threshold && angle < param.rotation_threshold)
                {
                    printf("Distance between KFs: %f Angle: %f Discarding keyframe\n", distance, angle);
                    continue;
                }      

                // calculate increment of angles
                keyFrame.calculateAngleIncrements(keyFrame2);                                       

                std::vector<KeyFrame> potentialKeyFrames = getPotentialLoopClosureKFs(&keyFrame);

                if (potentialKeyFrames.size() > 0)
                {
                    printf("Potential Key frames for loop closure found. Index: %i Count: %i\n", keyFrame.index, static_cast<int>(potentialKeyFrames.size()));
                    
                    for (int i=0; i < potentialKeyFrames.size(); i++)
                    {
                        KeyFrame compKeyFrame = potentialKeyFrames[i];
                        int sad = keyFrame.hist.calculateSAD(&compKeyFrame.hist);
                        printf("SAD for keyframe no %i against %i: %i\n", compKeyFrame.index, keyFrame.index, sad);
                    }
                }

            }

            printf("Distance between KFs: %f Angle: %f\n", distance, angle);

            keyFrames.push_back(keyFrame);            
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

float Mapping::getTranslationDistance(KeyFrame* keyFrame, KeyFrame* keyFrame2)
{
    // get euclidian distance between the two keyframes
    float f1, f2, f3;

    f1 = keyFrame->pose.val[0][3] - keyFrame2->pose.val[0][3];
    f2 = keyFrame->pose.val[1][3] - keyFrame2->pose.val[1][3];
    f3 = keyFrame->pose.val[2][3] - keyFrame2->pose.val[2][3];

    return fabs(sqrt(f1*f1 + f2*f2 + f3*f3));    
}

float Mapping::getRotationAngle(KeyFrame* keyFrame, KeyFrame* keyFrame2)
{
    // get rotation angle between the two keyframes    

    slam2::Matrix rA = keyFrame->pose.getMat(0, 0, 2, 2);
    slam2::Matrix rB = keyFrame2->pose.getMat(0, 0, 2, 2);

    slam2::Matrix rAT = rA.operator~();
    slam2::Matrix rAB = rAT.operator*(rB);

    // calculate trace
    float trace = rAB.val[0][0] + rAB.val[1][1] + rAB.val[2][2];

    if (trace > 3)
        trace = 3;

    float f1 = acos((trace - 1) / 2);
    float f2 = (f1 * 180) / M_PI;

    //printf("rotation angle: %f\n", f2);
    //printf("trace: %f, f1: %f f2:%f f5: %f f6: %f\n", trace, f1, f2, f5, f6);

    return f2;
}

std::vector<KeyFrame> Mapping::getPotentialLoopClosureKFs(KeyFrame* keyFrame)
{
    std::vector<KeyFrame> potentialKeyFrames;

    float xaccum, yaccum, zaccum;

    // xaccum = keyFrame->x_inc;
    // yaccum = keyFrame->y_inc;
    // zaccum = keyFrame->z_inc;

    for (int i=keyFrames.size(); i > 0; i--)
    {
        KeyFrame keyFrame2 = keyFrames[i - 1];

        xaccum += keyFrame2.x_inc;
        yaccum += keyFrame2.y_inc;
        zaccum += keyFrame2.z_inc;

        //printf("xaccum: %f yaccum: %f zaccum: %f index1: %i index2: %i\n", xaccum, yaccum, zaccum, keyFrame->index, keyFrame2.index);

        if (xaccum < param.angle_change_threshold && yaccum < param.angle_change_threshold
            && zaccum < param.angle_change_threshold)
            continue;

        printf("xaccum: %f yaccum: %f zaccum: %f index1: %i index2: %i\n", xaccum, yaccum, zaccum, keyFrame->index, keyFrame2.index);

        float translation = getTranslationDistance(keyFrame, &keyFrame2);

        if (translation > param.search_radius)
            continue;        

        printf("Translation Check: %f %i:%i\n", translation, keyFrame->index, keyFrame2.index);    

        float angle = getRotationAngle(&keyFrame2, keyFrame);

        if (angle > param.search_angle)
            continue;

        
        printf("Angle Check: %f %i:%i\n", angle, keyFrame->index, keyFrame2.index);    

        potentialKeyFrames.push_back(keyFrame2);
    }

    return potentialKeyFrames;
}