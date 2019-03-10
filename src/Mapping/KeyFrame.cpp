#include "KeyFrame.h"
#include <math.h>


KeyFrame::KeyFrame(int32_t index, int32_t width, int32_t height, slam2::Matrix pose)
{
    this->index = index;
    this->width = width;
    this->height = height;

	this->pose = slam2::Matrix(pose);
    x_inc = y_inc = z_inc = 0;
	calculateAngles();
	printf("angles x: %f y: %f z: %f index: %i\n", x, y, z, index);
	//hist = NULL;
}

KeyFrame::KeyFrame()
{
	//hist = NULL;
}

KeyFrame::~KeyFrame()
{
    // if (hist != NULL)
	// 	delete hist;
}

void KeyFrame::calculateAngles()
{
    float sy = sqrt(pose.val[0][0]* pose.val[0][0] + pose.val[1][1]*pose.val[1][1]);

    bool singular = sy < 1e-6;

    if (!singular)
    {
        x = atan2(pose.val[2][1], pose.val[2][2]);
        y = atan2(-pose.val[2][0], sy);
        z = atan2(pose.val[1][0], pose.val[0][0]);
    }
    else
    {
        x = atan2(pose.val[1][2], pose.val[1][1]);
        y = atan2(-pose.val[2][0], sy);
        z = 0;
    }

}

// void KeyFrame::calculateAngles()
// {
// 	// yaw = atan2(pose.val[2][1], pose.val[1][1]);
// 	// pitch = atan2(-pose.val[3][1], sqrtf(pose.val[3][2]*pose.val[3][2] + pose.val[3][3]*pose.val[3][3]));
// 	// roll = atan2(pose.val[3][2], pose.val[3][3])
// 	if (closeEnough(pose.val[0][2], -1.0f, 0.001))
// 	{
// 		x = 0;
// 		y = M_PI / 2;
// 		z = x + atan2(pose.val[1][0], pose.val[2][0]);
// 		return;
// 	}
// 	else if (closeEnough(pose.val[0][2], 1.0f, 0.001))
// 	{
// 		x = 0;
// 		y = -M_PI / 2;
// 		z = -x + atan2(-pose.val[1][0], -pose.val[2][0]);
// 		return;
// 	}
// 	else
// 	{
// 		float x1 = -asin(pose.val[0][2]);
// 		float x2 = M_PI - x1;
		
// 		float y1 = atan2(pose.val[1][2] / cos(x1), pose.val[2][2] / cos(x1));
// 		float y2 = atan2(pose.val[1][2] / cos(x2), pose.val[2][2] / cos(x2));
		
// 		float z1 = atan2(pose.val[0][1] / cos(x1), pose.val[0][0] / cos(x1));
// 		float z2 = atan2(pose.val[0][1] / cos(x1), pose.val[0][0] / cos(x1));

// 		if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2)))
// 		{
// 			x = x1;
// 			y = y1;
// 			z = z1;
// 		}
// 		else
// 		{
// 			x = x2;
// 			y = y2;
// 			z = z2;
// 		}
		
// 	}
// }

void KeyFrame::calculateAngleIncrements(KeyFrame prevKeyFrame)
{
	x_inc = angleDiff(prevKeyFrame.x, x);
	y_inc = angleDiff(prevKeyFrame.y, y);
	z_inc = angleDiff(prevKeyFrame.z, z);
}

float KeyFrame::angleDiff(float a1, float a2)
{
    if (std::signbit(a1) == std::signbit(a2))
        return a1 - a2;

    float a1adj, a2adj;

    if (a1 < 0)
        a1adj = -M_PI - a1;
    else
        a1adj = M_PI - a1;

    if (a2 < 0)
        a2adj = -M_PI - a2;
    else
        a2adj = M_PI - a2;

    float val1 = a1 - a2;
    float val2 = a1adj - a2adj;

    if (std::abs(val1) < std::abs(val2))
        return val1;
    else
        return val2;    
}


bool KeyFrame::closeEnough(const float& a, const float& b, const float& epsilon)
{
	return (epsilon > std::abs(a-b));
}

void KeyFrame::generateHistogram(Matches* matches)
{
	hist = Histogram(matches->p_matched, width, height, 50, 50);
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

