#pragma once

#include <cv.h>

class SLImage
{
    public:
        SLImage();
        SLImage(int width, int height, double timestamp, int index)
        {
            this->w = width;
            this->h = height;
            this->timestamp = timestamp;
            this->index = index;
        };
        cv::Mat image;
        cv::Mat imageColor;
        int w, h, index; 
        double timestamp;
        float exposure_time;

        uint8_t* getImageArray()
        {
            uint8_t* img_data  = (uint8_t*)malloc(w*h*sizeof(uint8_t));
            int32_t k=0;

            for (int32_t v=0; v<h; v++) {
            for (int32_t u=0; u<w; u++) {
                img_data[k]  = image.at<uint8_t>(v,u); 
                k++;
            }
            }

            return img_data;
        }
};