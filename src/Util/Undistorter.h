#pragma once

#include <string>
#include "NumType.h"
#include <cv.h>
#include "SLImage.h"

class Undistorter
{
public:
    void loadFromFile(std::string configFileName);
    cv::Mat getCameraMatrix();
    void processPhotometricFrame();
    SLImage* undistort(cv::Mat input, double timestamp);
    int getOutputWidth();
    int getOutputHeight();
    float getBaseline();

private:
    std::string configFileName;
    int wOrig, hOrig;
    int wOut, hOut;
    VecX parsOrg;
    float baseLine;

    bool valid;
	bool passthrough;

    Mat33 K;

	float* remapX;
	float* remapY;

	void makeOptimalK_crop();
	void makeOptimalK_full();
    
    float* convertToFloat(cv::Mat image);
    cv::Mat convertFromFloat(float* data, int width, int height);

    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;
};