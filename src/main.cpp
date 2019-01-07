
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

std::string source = "";
std::string calib = "";

int width = 0;
int height = 0;

float playbackSpeed = 1.0;

void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	while(true) pause();
}


int main( int argc, char** argv )
{
    calib = "/home/lbarnett/development/odometry/00/param/camera.txt";
    source = "/home/lbarnett/development/odometry/00";

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

    ImageReader* imageReader = new ImageReader(false);
    ImageReader* imageReaderRight = new ImageReader(false);

    imageReader->loadImage(reader->getImageFilename(0));

    width = imageReader->getImageWidth();
    height = imageReader->getImageHeight();

    if (width <1 || height < 1)
    {
        printf("Invalid width or height. Exiting.....");
        return 0;
    }

    printf("Setting width to %i and height to %i\n", width, height);

    SlamViewer* slamViewer = new SlamViewer(width, height);

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

        for (int i=0; i < imageCount; i++){

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

            imageReader->loadImage(reader->getImageFilename(i));
            imageReaderRight->loadImage(readerRight->getImageFilename(i));

            if (!skipFrame)
                slamViewer->pushLiveImageFrame(imageReader->getImage(), imageReaderRight->getImage());
        }

    });


    if(slamViewer != 0)
        slamViewer->run();

    runthread.join();

    delete imageReader;
    delete reader;
    return 0;
}