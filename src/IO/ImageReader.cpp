#include "ImageReader.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

ImageReader::ImageReader(bool displayImage, std::string paramFile){
    this->displayImage = displayImage;
	udist = new Undistorter();
    udist->loadFromFile(paramFile);
}

ImageReader::~ImageReader()
{
	delete udist;
}

int ImageReader::getImageWidth()
{
	if (image.empty())
		return 0;
	else
		return image.cols;
}


int ImageReader::getImageHeight()
{
	if (image.empty())
		return 0;
	else
		return image.rows;
}

bool ImageReader::loadImage(std::string fileName)
{
    image = cv::imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
	if(image.rows*image.cols==0)
	{
		printf("cv::imread could not read image %s! this may segfault. \n", fileName.c_str());
		return false;
	}

	
	if(image.type() != CV_8U)
	{
		printf("cv::imread did something strange! this may segfault. \n");
		return false;
	}
		
    if (displayImage)
    {
		printf("rows: %i, cols: %i channels: %i type: %i\n", image.rows, image.cols, image.channels(), image.type());
	    cv::namedWindow( "Display window", 0x00000001 );// Create a window for display.
        cv::imshow( "Display window", image);                   // Show our image inside it.

        cv::waitKey(0);
    }
}

cv::Mat ImageReader::getImage()
{
    return image;
}

int ImageReader::getUDistImageWidth()
{
	return udist->getOutputWidth();
}

int ImageReader::getUDistImageHeight()
{
	return udist->getOutputHeight();
}

SLImage* ImageReader::getUndistortedImage(double timestamp)
{
	return udist->undistort(image, timestamp);
}