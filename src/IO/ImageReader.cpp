#include "ImageReader.h"

ImageReader::ImageReader(bool displayImage){
    this->displayImage = displayImage;
}

ImageReader::~ImageReader()
{

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