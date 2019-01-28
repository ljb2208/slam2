#include "KeyFrame.h"


KeyFrame::KeyFrame()
{
    
}

void KeyFrame::generateDepthInfo()
{
    for (int i=0; i < p_matched_p.size(); i++)
    {
        float ul,ur;

        ul = p_matched_p[i].u1c;
        ur = p_matched_p[i].u2c;

        

    }
}

