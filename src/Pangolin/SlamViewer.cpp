#include "SlamViewer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "IO/ImageReader.h"

SlamViewer::SlamViewer(int width, int height)
{
    this->width = width;
    this->height = height;
}

SlamViewer::~SlamViewer()
{}


void SlamViewer::run()
{
    printf("Start Slam Viewer\n");

    running = true;
    videoImageChanged = false;

    internalVideoImg.create(height, width, CV_8U);
    internalVideoImgRight.create(height, width, CV_8U);
    
	pangolin::CreateWindowAndBind("SLAM Viewer",2*width,2*height);
	const int UI_WIDTH = 180;

	glEnable(GL_DEPTH_TEST);

    pangolin::View& d_video = pangolin::Display("imgVideo")
	    .SetAspect((float)width/height);

    pangolin::View& d_videoRight = pangolin::Display("imgVideoRight")
	    .SetAspect((float)width/height);

    pangolin::GlTexture texVideo(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture texVideoRight(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


    pangolin::CreateDisplay()
		  .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
		  .SetLayout(pangolin::LayoutEqual)
		  .AddDisplay(d_video)
          .AddDisplay(d_videoRight);
	
    // Default hooks for exiting (Esc) and fullscreen (tab).
	while( !pangolin::ShouldQuit() && running )
	{
        // Clear entire screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        openImagesMutex.lock();

		if(videoImageChanged) {
		    texVideo.Upload(internalVideoImg.data, GL_BGR, GL_UNSIGNED_BYTE);
            texVideoRight.Upload(internalVideoImgRight.data, GL_BGR, GL_UNSIGNED_BYTE);
		}

		videoImageChanged=false;
		
		openImagesMutex.unlock();

		d_video.Activate();
		glColor4f(1.0f,1.0f,1.0f,1.0f);
		texVideo.RenderToViewportFlipY();

		d_videoRight.Activate();
		glColor4f(1.0f,1.0f,1.0f,1.0f);
		texVideoRight.RenderToViewportFlipY();

        // Swap frames and Process Events
		pangolin::FinishFrame();
    }

    exit(1);
}

void SlamViewer::pushLiveImageFrame(cv::Mat image, cv::Mat imageRight)
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex);

    internalVideoImg = image;
    internalVideoImgRight = imageRight;
    //cv::cvtColor(image, internalVideoImg, cv::COLOR_GRAY2RGB);
    //cv::cvtColor(imageRight, internalVideoImgRight, cv::COLOR_GRAY2RGB);

    videoImageChanged = true;
}

void SlamViewer::close()
{
    running = false;
}

void SlamViewer::join()
{
    runThread.join();
    printf("Joined slam viewer thread\n");
}


void SlamViewer::VideoSample(const std::string uri)
{
    // Setup Video Source
    pangolin::VideoInput video(uri);
    const pangolin::PixelFormat vid_fmt = video.PixFormat();
    const unsigned w = video.Width();
    const unsigned h = video.Height();

    printf("Video w: %i h: %i\n", w, h);

    // Work out appropriate GL channel and format options
    GLint glformat = GL_RGB;
    GLenum gltype = GL_UNSIGNED_BYTE;
        
    // Create OpenGL window
    pangolin::CreateWindowAndBind("Main",w,h);

    // Create viewport for video with fixed aspect
    pangolin::View& vVideo = pangolin::Display("Video").SetAspect((float)w/h);

    // OpenGl Texture for video frame.
    pangolin::GlTexture texVideo(w,h,glformat,false,0,glformat,gltype);

    unsigned char* img = new unsigned char[video.SizeBytes()];

    for(int frame=0; !pangolin::ShouldQuit(); ++frame)
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        cv::Mat image = cv::imread(uri, 1);
        texVideo.Upload( image.data, glformat, gltype );

        //printf("Image format: %i, type: %i\n", glformat, gltype);

        // Activate video viewport and render texture
        vVideo.Activate();
        texVideo.RenderToViewportFlipY();

        // Swap back buffer with front and process window events
        pangolin::FinishFrame();
    }

    delete[] img;
}