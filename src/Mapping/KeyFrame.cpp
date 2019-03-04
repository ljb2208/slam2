#include "KeyFrame.h"


KeyFrame::KeyFrame(int32_t index, int32_t width, int32_t height)
{
    this->index = index;
    this->width = width;
    this->height = height;
}

KeyFrame::KeyFrame()
{
    
}

void KeyFrame::generateDepthInfo()
{
    for (int i=0; i < p_matched.size(); i++)
    {

    }
}

Sophus::SE3 KeyFrame::getSE3()
{
    Vec3d trans;

	trans(0) = pose.val[0][3];
	trans(1) = pose.val[1][3];
	trans(2) = pose.val[2][3];

	Mat33 rot;

	rot(0, 0) = pose.val[0][0];
	rot(0, 1) = pose.val[0][1];
	rot(0, 2) = pose.val[0][2];

	rot(1, 0) = pose.val[1][0];
	rot(1, 1) = pose.val[1][1];
	rot(1, 2) = pose.val[1][2];

	rot(2, 0) = pose.val[2][0];
	rot(2, 1) = pose.val[2][1];
	rot(2, 2) = pose.val[2][2];

	return SE3(rot, trans);
}

