#include "SlamViewer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "IO/ImageReader.h"
#include "Util/Settings.h"
#include "IO/DataSetReader.h"

SlamViewer::SlamViewer(int width, int height)
{
    this->width = width;
    this->height = height;

    //settings_showTrajectory = Settings::settings_showTrajectory;
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

    // 3D visualization
	pangolin::OpenGlRenderState Visualization3D_camera(
		pangolin::ProjectionMatrix(width,height,400,400,width/2,height/2,0.1,1000),
		pangolin::ModelViewLookAt(-0,-5,-10, 0,0,0, pangolin::AxisNegY)
		);

	pangolin::View& Visualization3D_display = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -width/(float)height)
		.SetHandler(new pangolin::Handler3D(Visualization3D_camera));

    pangolin::View& d_kfDepth = pangolin::Display("imgKFDepth")
	    .SetAspect(width/(float)height);

    pangolin::View& d_video = pangolin::Display("imgVideo")
	    .SetAspect((float)width/height);

    pangolin::View& d_videoRight = pangolin::Display("imgVideoRight")
	    .SetAspect((float)width/height);

	pangolin::GlTexture texKFDepth(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture texVideo(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture texVideoRight(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


    pangolin::CreateDisplay()
		  .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
		  .SetLayout(pangolin::LayoutEqual)
          .AddDisplay(d_kfDepth)
		  .AddDisplay(d_video)
          .AddDisplay(d_videoRight);
	

    float yellow[3] = {1,1,0};
    float blue[3] = {0,0,1};
    

    // Default hooks for exiting (Esc) and fullscreen (tab).
	while( !pangolin::ShouldQuit() && running )
	{
        // Clear entire screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        
        if(setting_render_display3D)
		{            
            
			// Activate efficiently by object
			Visualization3D_display.Activate(Visualization3D_camera);
			boost::unique_lock<boost::mutex> lk3d(model3DMutex);
			
			int refreshed=0;

            /*
			for(KeyFrameDisplay* fh : keyframes)
			{				
				if(this->settings_showKFCameras) fh->drawCam(1,blue,0.1);

				refreshed =+ (int)(fh->refreshPC(refreshed < 10, this->settings_scaledVarTH, this->settings_absVarTH,
						this->settings_pointCloudMode, this->settings_minRelBS, this->settings_sparsity));
				fh->drawPC(1);

			}
            

            for(int i = 0; i < matrix_result.size(); i++)
            {

                KeyFrameDisplay* fh = new KeyFrameDisplay;
                fh->drawGTCam(matrix_result[i], 5, yellow, 0.1);
                delete(fh);
            }            

            
			if(this->settings_showCurrentCamera) currentCam->drawCam(2,0,0.2);
            */
			drawConstraints();
            
            
			lk3d.unlock();
		}
        

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

void SlamViewer::pushKeyFrame(KeyFrame keyFrame)
{
    boost::unique_lock<boost::mutex> lk(model3DMutex);
    keyFrames.push_back(keyFrame);
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

void SlamViewer::drawConstraints()
{
    if(settings_showAllConstraints)
	{
		// draw constraints
		glLineWidth(1);
		glBegin(GL_LINES);

		glColor3f(0,1,0);
		glBegin(GL_LINES);
		for(unsigned int i=0;i<connections.size();i++)
		{
			if(connections[i].to == 0 || connections[i].from==0) continue;
			int nAct = connections[i].bwdAct + connections[i].fwdAct;
			int nMarg = connections[i].bwdMarg + connections[i].fwdMarg;
			if(nAct==0 && nMarg>0  )
			{
				Sophus::Vector3f t = connections[i].from->camToWorld.translation().cast<float>();
				glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
				t = connections[i].to->camToWorld.translation().cast<float>();
				glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
			}
		}
		glEnd();
	}

	if(settings_showActiveConstraints)
	{
		glLineWidth(3);
		glColor3f(0,0,1);
		glBegin(GL_LINES);
		for(unsigned int i=0;i<connections.size();i++)
		{
			if(connections[i].to == 0 || connections[i].from==0) continue;
			int nAct = connections[i].bwdAct + connections[i].fwdAct;

			if(nAct>0)
			{
				Sophus::Vector3f t = connections[i].from->camToWorld.translation().cast<float>();
				glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
				t = connections[i].to->camToWorld.translation().cast<float>();
				glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
			}
		}
		glEnd();
	}

	if(settings_showTrajectory)
	{
        float colorRed[3] = {1,0,0};
		glColor3f(colorRed[0],colorRed[1],colorRed[2]);
		glLineWidth(3);

		glBegin(GL_LINE_STRIP);        

		for(unsigned int i=0;i<keyFrames.size();i++)
		{                        
            float f1, f2, f3;

            if (keyFrames[i].pose.val != 0)
            {
                f1 = keyFrames[i].pose.val[0][3];
                f2 = keyFrames[i].pose.val[1][3];
                f3 = keyFrames[i].pose.val[2][3];
                glVertex3f(f1, f2, f3);					            
            }

		}
		glEnd();        
	}

    if(settings_showGroundTruth)
	{
        float colorYellow[3] = {1,1,0};
		glColor3f(colorYellow[0],colorYellow[1],colorYellow[2]);
		glLineWidth(3);

		glBegin(GL_LINE_STRIP);        

		for(unsigned int i=0;i<matrix_result.size();i++)
		{                        
            float f1, f2, f3;
            
            f1 = matrix_result[i](0, 3);
            f2 = matrix_result[i](1, 3);
            f3 = matrix_result[i](2, 3);
            glVertex3f(f1, f2, f3);					                        

		}
		glEnd();        
	}

	if(settings_showFullTrajectory)
	{
		float colorGreen[3] = {0,1,0};
		glColor3f(colorGreen[0],colorGreen[1],colorGreen[2]);
		glLineWidth(3);

		glBegin(GL_LINE_STRIP);
		for(unsigned int i=0;i<allFramePoses.size();i++)
		{
			glVertex3f((float)allFramePoses[i][0],
					(float)allFramePoses[i][1],
					(float)allFramePoses[i][2]);
		}
		glEnd();
	}
}