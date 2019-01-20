
#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <chrono>

#include "IO/DataSetReader.h"
#include "IO/ImageReader.h"
#include "Pangolin/SlamViewer.h"
#include "Util/Undistorter.h"
#include "Odometry/Odometry.h"
#include "cv.h"
#include <opencv2/imgproc.hpp>

std::string source = "";
std::string calib = "";
std::string param = "/home/ljb2208/development/odometry/00/param/camera.txt";

int width = 0;
int height = 0;

float playbackSpeed = 1.0;  //1.0

bool running = true;

void my_exit_handler(int s)
{
    running = false;
	printf("Exiting due to user %d\n",s);
	
}

void exitThread()
{
    printf("Exit thread...\n");
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	while(true) pause();
}


int main( int argc, char** argv )
{
    calib = "/home/ljb2208/development/odometry/00/param/camera.txt";
    source = "/home/ljb2208/development/odometry/00";

    // hook crtl+C.
	boost::thread exThread = boost::thread(exitThread);
   
    ImageFolderReader* reader = new ImageFolderReader(source+"/image_0", calib);
    ImageFolderReader* readerRight = new ImageFolderReader(source+"/image_1", calib);
    int imageCount = reader->getNumImages();

    if (imageCount < 1)
    {
        printf("No images found! Exiting...");
        return 0;
    }

    if (imageCount != readerRight->getNumImages())
    {
        printf("Differing number of left and right images. Exiting....");
        return 0;
    }

    ImageReader* imageReader = new ImageReader(false, param);
    ImageReader* imageReaderRight = new ImageReader(false, param);

    imageReader->loadImage(reader->getImageFilename(0));

    width = imageReader->getUDistImageWidth();
    height = imageReader->getUDistImageHeight();

    if (width <1 || height < 1)
    {
        printf("Invalid width or height. Exiting.....");
        return 0;
    }

    printf("Setting width to %i and height to %i\n", width, height);

    cv::Mat cameraMatrix = imageReader->getCameraMatrix();
    float baseLine = imageReader->getBaseline();

    SlamViewer* slamViewer = new SlamViewer(width, height);
    Odometry* odom = new Odometry(slamViewer, cameraMatrix, baseLine);

    std::thread runthread([&]() {

        std::vector<int> idsToPlay;				
        std::vector<double> timesToPlayAt;

        printf("Getting timestamps\n");

        for (int y=0; y < reader->getNumImages(); y++)
        {
            idsToPlay.push_back(y);

            if (y == 0)
            {
                timesToPlayAt.push_back((double)0);
            }
            else
            {
                double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
                double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
                timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
            }
        }

        printf("Timestamps created\n");

        // timing
        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset=0;

        if (imageCount > 100)
            imageCount = 100;

        for (int i=0; i < imageCount; i++){

            if (!running)
                exit(1);
            bool skipFrame=false;
            if(playbackSpeed!=0)
            {
                struct timeval tv_now; gettimeofday(&tv_now, NULL);
                double sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));

                if(sSinceStart < timesToPlayAt[i])
                    usleep((int)((timesToPlayAt[i]-sSinceStart)*1000*1000));
                else if(sSinceStart > timesToPlayAt[i]+0.5+0.1*(i%2))
                {
                    printf("SKIPFRAME %d (play at %f, now it is %f)!\n", i, timesToPlayAt[i], sSinceStart);
                    skipFrame=true;
                }
            }

            double ts = timesToPlayAt[i];

            
            imageReader->loadImage(reader->getImageFilename(i));
            imageReaderRight->loadImage(readerRight->getImageFilename(i));                        
            
            if (!skipFrame)
            {
                SLImage* sli_left = imageReader->getUndistortedImage(ts);
                SLImage* sli_right = imageReaderRight->getUndistortedImage(ts);
                odom->addStereoFrames(sli_left, sli_right);

                delete sli_left;
                delete sli_right;

            }
            
        }

        odom->timer->outputAll();

    });


    if(slamViewer != 0)
        slamViewer->run();

    runthread.join();

    printf("output timers\n");
    

    delete imageReader;
    delete imageReaderRight;
    delete reader;
    delete slamViewer;
    delete odom;
    return 0;
}