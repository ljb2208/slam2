#include "Undistorter.h"

#include <sstream>
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <iterator>
#include "NumType.h"
#include "Settings.h"

#include <opencv2/opencv.hpp>

void Undistorter::loadFromFile(std::string configFileName)
{
    this->configFileName = configFileName;

    std::string prefix = "";
    float outputCalibration[5];
    parsOrg = VecX(5);
    std::string ln1,ln2,ln3,ln4,ln5;

    printf("Reading Calibration from file %s\n",configFileName.c_str());

    std::ifstream cfgfile(configFileName.c_str());
	if (!cfgfile.good())
	{
		cfgfile.close();
		printf("Config file not found....\n");
		cfgfile.close();
		return;
	}

	std::getline(cfgfile,ln1);
	std::getline(cfgfile,ln2);
    std::getline(cfgfile,ln3);
    std::getline(cfgfile,ln4);
    std::getline(cfgfile,ln5);

    char buf[1000];
    snprintf(buf, 1000, "%s%%lf %%lf %%lf %%lf %%lf", prefix.c_str());

    if(std::sscanf(ln1.c_str(), buf, &parsOrg[0], &parsOrg[1], &parsOrg[2], &parsOrg[3], &parsOrg[4]) == 5 &&
            std::sscanf(ln2.c_str(), "%d %d", &wOrig, &hOrig) == 2)
    {
        printf("Input resolution: %d %d\n",wOrig, hOrig);
        printf("In: %f %f %f %f %f\n",
                parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3], parsOrg[4]);
    }
    else
    {
        printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName.c_str());
        cfgfile.close();
        return;
    }

    // get output param
	if(ln3 == "crop")
	{
		outputCalibration[0] = -1;
        printf("Out: Rectify Crop\n");
	}
	else if(ln3 == "full")
	{
		outputCalibration[0] = -2;
        printf("Out: Rectify Full\n");
	}
	else if(ln3 == "none")
	{
		outputCalibration[0] = -3;
        printf("Out: No Rectification\n");
	}
	else if(std::sscanf(ln3.c_str(), "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
	{
		printf("Out: %f %f %f %f %f\n",
				outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3], outputCalibration[4]);

	}
	else
	{
		printf("Out: Failed to Read Output pars... not rectifying.\n");
		return;
	}

    // get output width and height
    if(std::sscanf(ln4.c_str(), "%d %d", &wOut, &hOut) == 2)
	{
        if(benchmarkSetting_width != 0)
        {
			wOut = benchmarkSetting_width;
            if(outputCalibration[0] == -3)
                outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
        }
        if(benchmarkSetting_height != 0)
        {
			hOut = benchmarkSetting_height;
            if(outputCalibration[0] == -3)
                outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
        }

		printf("Output resolution: %d %d\n",wOut, hOut);
	}
	else
	{
		printf("Out: Failed to Read Output resolution... not rectifying.\n");
		wOut = wOrig;
    }

    // get stereo camera baseline
    if(std::sscanf(ln5.c_str(), "%f", &baseLine) == 1)
    {
        printf("Baseline: %f \n", baseLine);
    }
    else
    {
        printf("Failed to Read Baseline... can not do stereo. \n");
    }

    remapX = new float[wOut*hOut];
    remapY = new float[wOut*hOut];

    printf("out w: %i h: %i output calib: %f\n", wOut, hOut, outputCalibration[0]);

	if(outputCalibration[0] == -1)
		makeOptimalK_crop();
	else if(outputCalibration[0] == -2)
		makeOptimalK_full();
	else if(outputCalibration[0] == -3)
	{
		if(wOut != wOrig || hOut != hOrig)
		{
			printf("ERROR: rectification mode none requires input and output dimenstions to match!\n\n");
			exit(1);
		}
		K.setIdentity();
        K(0,0) = parsOrg[0];
        K(1,1) = parsOrg[1];
        K(0,2) = parsOrg[2];
        K(1,2) = parsOrg[3];
		passthrough = true;
	}
	else
	{


        if(outputCalibration[2] > 1 || outputCalibration[3] > 1)
        {
            printf("\n\n\nWARNING: given output calibration (%f %f %f %f) seems wrong. It needs to be relative to image width / height!\n\n\n",
                   outputCalibration[0],outputCalibration[1],outputCalibration[2],outputCalibration[3]);
        }


		K.setIdentity();
        K(0,0) = outputCalibration[0] * wOut;
        K(1,1) = outputCalibration[1] * hOut;
        K(0,2) = outputCalibration[2] * wOut - 0.5;
        K(1,2) = outputCalibration[3] * hOut - 0.5;
	}

	if(benchmarkSetting_fxfyfac != 0)
	{
		K(0,0) = fmax(benchmarkSetting_fxfyfac, (float)K(0,0));
		K(1,1) = fmax(benchmarkSetting_fxfyfac, (float)K(1,1));
        passthrough = false; // cannot pass through when fx / fy have been overwritten.
	}

	for(int y=0;y<hOut;y++)
		for(int x=0;x<wOut;x++)
		{
			remapX[x+y*wOut] = x;
			remapY[x+y*wOut] = y;
		}

	distortCoordinates(remapX, remapY, remapX, remapY, hOut*wOut);


	for(int y=0;y<hOut;y++)
		for(int x=0;x<wOut;x++)
		{
			// make rounding resistant.
			float ix = remapX[x+y*wOut];
			float iy = remapY[x+y*wOut];

			if(ix == 0) ix = 0.001;
			if(iy == 0) iy = 0.001;
			if(ix == wOrig-1) ix = wOrig-1.001;
			if(iy == hOrig-1) ix = hOrig-1.001;

			if(ix > 0 && iy > 0 && ix < wOrig-1 &&  iy < wOrig-1)
			{
				remapX[x+y*wOut] = ix;
				remapY[x+y*wOut] = iy;
			}
			else
			{
				remapX[x+y*wOut] = -1;
				remapY[x+y*wOut] = -1;
			}
		}

	valid = true;

	printf("\nRectified Kamera Matrix:\n");
	std::cout << K << "\n\n";

}

void Undistorter::makeOptimalK_crop()
{
    printf("finding CROP optimal new model!\n");
	K.setIdentity();

	// 1. stretch the center lines as far as possible, to get initial coarse quess.
	float* tgX = new float[100000];
	float* tgY = new float[100000];
	float minX = 0;
	float maxX = 0;
	float minY = 0;
	float maxY = 0;

	for(int x=0; x<100000;x++)
	{tgX[x] = (x-50000.0f) / 10000.0f; tgY[x] = 0;}
	distortCoordinates(tgX, tgY,tgX, tgY,100000);
	for(int x=0; x<100000;x++)
	{
		if(tgX[x] > 0 && tgX[x] < wOrig-1)
		{
			if(minX==0) minX = (x-50000.0f) / 10000.0f;
			maxX = (x-50000.0f) / 10000.0f;
		}
	}
	for(int y=0; y<100000;y++)
	{tgY[y] = (y-50000.0f) / 10000.0f; tgX[y] = 0;}
	distortCoordinates(tgX, tgY,tgX, tgY,100000);
	for(int y=0; y<100000;y++)
	{
		if(tgY[y] > 0 && tgY[y] < hOrig-1)
		{
			if(minY==0) minY = (y-50000.0f) / 10000.0f;
			maxY = (y-50000.0f) / 10000.0f;
		}
	}
	delete[] tgX;
	delete[] tgY;

	minX *= 1.01;
	maxX *= 1.01;
	minY *= 1.01;
	maxY *= 1.01;



	printf("initial range: x: %.4f - %.4f; y: %.4f - %.4f!\n", minX, maxX, minY, maxY);



	// 2. while there are invalid pixels at the border: shrink square at the side that has invalid pixels,
	// if several to choose from, shrink the wider dimension.
	bool oobLeft=true, oobRight=true, oobTop=true, oobBottom=true;
	int iteration=0;
	while(oobLeft || oobRight || oobTop || oobBottom)
	{
		oobLeft=oobRight=oobTop=oobBottom=false;
		for(int y=0;y<hOut;y++)
		{
			remapX[y*2] = minX;
			remapX[y*2+1] = maxX;
			remapY[y*2] = remapY[y*2+1] = minY + (maxY-minY) * (float)y / ((float)hOut-1.0f);
		}
		distortCoordinates(remapX, remapY,remapX, remapY,2*hOut);
		for(int y=0;y<hOut;y++)
		{
			if(!(remapX[2*y] > 0 && remapX[2*y] < wOrig-1))
				oobLeft = true;
			if(!(remapX[2*y+1] > 0 && remapX[2*y+1] < wOrig-1))
				oobRight = true;
		}



		for(int x=0;x<wOut;x++)
		{
			remapY[x*2] = minY;
			remapY[x*2+1] = maxY;
			remapX[x*2] = remapX[x*2+1] = minX + (maxX-minX) * (float)x / ((float)wOut-1.0f);
		}
		distortCoordinates(remapX, remapY,remapX, remapY,2*wOut);


		for(int x=0;x<wOut;x++)
		{
			if(!(remapY[2*x] > 0 && remapY[2*x] < hOrig-1))
				oobTop = true;
			if(!(remapY[2*x+1] > 0 && remapY[2*x+1] < hOrig-1))
				oobBottom = true;
		}


		if((oobLeft || oobRight) && (oobTop || oobBottom))
		{
			if((maxX-minX) > (maxY-minY))
				oobBottom = oobTop = false;	// only shrink left/right
			else
				oobLeft = oobRight = false; // only shrink top/bottom
		}

		if(oobLeft) minX *= 0.995;
		if(oobRight) maxX *= 0.995;
		if(oobTop) minY *= 0.995;
		if(oobBottom) maxY *= 0.995;

		iteration++;


		printf("iteration %05d: range: x: %.4f - %.4f; y: %.4f - %.4f!\n", iteration,  minX, maxX, minY, maxY);
		if(iteration > 500)
		{
			printf("FAILED TO COMPUTE GOOD CAMERA MATRIX - SOMETHING IS SERIOUSLY WRONG. ABORTING \n");
			exit(1);
		}
	}

	K(0,0) = ((float)wOut-1.0f)/(maxX-minX);
	K(1,1) = ((float)hOut-1.0f)/(maxY-minY);
	K(0,2) = -minX*K(0,0);
	K(1,2) = -minY*K(1,1);
}
	
void Undistorter::makeOptimalK_full()
{

}

void Undistorter::distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const
{
    // current camera parameters
    float fx = parsOrg[0];
    float fy = parsOrg[1];
    float cx = parsOrg[2];
    float cy = parsOrg[3];

	float ofx = K(0,0);
	float ofy = K(1,1);
	float ocx = K(0,2);
	float ocy = K(1,2);

	for(int i=0;i<n;i++)
	{
		float x = in_x[i];
		float y = in_y[i];
		float ix = (x - ocx) / ofx;
		float iy = (y - ocy) / ofy;
		ix = fx*ix+cx;
		iy = fy*iy+cy;
		out_x[i] = ix;
		out_y[i] = iy;
	}
}

cv::Mat Undistorter::getCameraMatrix()
{
    cv::Mat cam;
    cam.create(3, 3, CV_32F);

    for (int i=0; i < 3; i++)
    {
        cam.at<float>(i, 0) = K(i, 0);
        cam.at<float>(i, 1) = K(i, 1);
        cam.at<float>(i, 2) = K(i, 2);
    }

    return cam;
}

SLImage* Undistorter::undistort(cv::Mat input, double timestamp)
{
    SLImage* result = new SLImage(wOut, hOut, timestamp);

	float* in_data = convertToFloat(input);
	float* out_data = new float[wOut * hOut];
    //float* out_data = result->image.data;
	//float* in_data = flimage.data;

    for(int idx = wOut*hOut-1;idx>=0;idx--)
    {
        // get interp. values
        float xx = remapX[idx];
        float yy = remapY[idx];


        if(xx<0)
            out_data[idx] = 0;
        else
        {
            // get integer and rational parts
            int xxi = xx;
            int yyi = yy;
            xx -= xxi;
            yy -= yyi;
            float xxyy = xx*yy;

            // get array base pointer
            const float* src = in_data + xxi + yyi * wOrig;

            // interpolate (bilinear)
            out_data[idx] =  xxyy * src[1+wOrig]
                                + (yy-xxyy) * src[wOrig]
                                + (xx-xxyy) * src[1]
                                + (1-xx-yy+xxyy) * src[0];
        }
    }

	result->image = convertFromFloat(out_data, wOut, hOut);
	imwrite("result.png", result->image);

    return result;
}


    float* Undistorter::convertToFloat(cv::Mat image)
	{
		int sz = image.rows * image.cols;

		float* result = new float[sz];

		int x = 0;

		for (int i=0; i < image.rows; i++)
		{
			for (int y=0; y < image.cols; y++)
			{
				result[x] = (float) image.at<uchar>(i, y);
				x++;
			}
		}

		return result;
	}

    cv::Mat Undistorter::convertFromFloat(float* data, int width, int height)
	{
		cv::Mat result = cv::Mat(height, width, CV_8UC1);

		int x = 0;
		for (int i=0; i < height; i++)
		{
			for (int y=0; y < width; y++)
			{
				result.at<uchar>(i, y) = (uchar)data[x];
				x++;
			}
		}

		return result;
	}

	int Undistorter::getOutputWidth()
	{
		return wOut;
	}

    int Undistorter::getOutputHeight()
	{
		return hOut;
	}