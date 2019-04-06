#include "SlamViewer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "IO/ImageReader.h"
#include "Util/Settings.h"
#include "IO/DataSetReader.h"


SlamViewer::SlamViewer(int width, int height, int imageOffset)
{
    this->width = width;
    this->height = height;
	this->imageOffset = imageOffset;
	currentImageId = 0;

	settings_pointCloudMode = 1;
	settings_sparsity = 1;
	settings_scaledVarTH = 0.001;
	settings_absVarTH = 0.001;

	keyFrameFile.open("keyFrames.txt", std::ios::out | std::ios::trunc);

    //settings_showTrajectory = Settings::settings_showTrajectory;
}

SlamViewer::~SlamViewer()
{}


void SlamViewer::run()
{
    printf("Start Slam Viewer\n");
	

    running = true;
    videoImageChanged = false;

	if (settings_showGroundTruth)
	{	
		ImageFolderReader* reader = new ImageFolderReader();
		groundTruth = reader->getGroundTruth(imageOffset);
		delete reader;
	}

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
		.SetBounds(0.0, 1.0f, 0 /*pangolin::Attach::Pix(UI_WIDTH)*/, 1.0f, -width/(float)height)
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

            
			for(KeyFrameDisplay* fh : keyFrames)
			{				
				//if(this->settings_showKFCameras) 
				
				fh->drawCam(1,blue,0.1);

				refreshed =+ (int)(fh->refreshPC(refreshed < 10, this->settings_scaledVarTH, this->settings_absVarTH,
						this->settings_pointCloudMode, this->settings_minRelBS, this->settings_sparsity));
				fh->drawPC(1);

			}
            
			/*
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

	keyFrameFile.close();

    exit(1);
}

void SlamViewer::pushLiveImageFrame(cv::Mat image, cv::Mat imageRight, int imageId)
{
    boost::unique_lock<boost::mutex> lk(openImagesMutex);

    internalVideoImg = image;
    internalVideoImgRight = imageRight;
    //cv::cvtColor(image, internalVideoImg, cv::COLOR_GRAY2RGB);
    //cv::cvtColor(imageRight, internalVideoImgRight, cv::COLOR_GRAY2RGB);

	currentImageId = imageId;
    videoImageChanged = true;
}

void SlamViewer::pushKeyFrame(KeyFrame keyFrame)
{
	KeyFrameDisplay* disp = new KeyFrameDisplay(keyFrame, fx, fy, cx, cy);

	Sophus::Matrix4f m = disp->camToWorld.matrix().cast<float>();	

	keyFrameFile << "KeyFrame: " << keyFrame.index << "\n";		
	keyFrameFile << "CamToWorld: \n" << m << "\n\n"; 
	
    boost::unique_lock<boost::mutex> lk(model3DMutex);
    keyFrames.push_back(disp);
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

			slam2::Matrix pose = keyFrames[i]->getPose();

            if (pose.val != 0)
            {
                f1 = pose.val[0][3];
                f2 = pose.val[1][3];
                f3 = pose.val[2][3];
                glVertex3f(f1, f2, f3);					            
            }

		}
		glEnd();        
	}

    if(settings_showGroundTruth)
	{
		// float f_adj1, f_adj2, f_adj3;
		// f_adj1 = f_adj2 = f_adj3 = 0;

		// if (imageOffset != 0)
		// {
		// 	f_adj1 = groundTruth[imageOffset].val[0][3];
		// 	f_adj2 = groundTruth[imageOffset].val[1][3];
		// 	f_adj3 = groundTruth[imageOffset].val[2][3];
		// }

		// printf("orig f1: %f f2: %f f3: %f\n", f_adj1, f_adj2, f_adj3);

        float colorYellow[3] = {1,1,0};
		glColor3f(colorYellow[0],colorYellow[1],colorYellow[2]);
		glLineWidth(3);

		glBegin(GL_LINE_STRIP);        

		for(unsigned int i=0;i<groundTruth.size();i++)
		{    
			// if (i > (currentImageId + imageOffset))
			// 	break;

			if (i > (currentImageId))
				break;

            float f1, f2, f3;
            
            f1 = groundTruth[i].val[0][3];// - f_adj1;
            f2 = groundTruth[i].val[1][3];// - f_adj2;
            f3 = groundTruth[i].val[2][3];// - f_adj3;
            glVertex3f(f1, f2, f3);					

			// printf("f1: %f f2: %f f3: %f\n", f1, f2, f3);                        

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


void SlamViewer::setCalibration(float fx, float fy, float cx, float cy)
{
	this->fx = fx;
	this->fy = fy;
	this->cx = cx;
	this->cy = cy;
}