#pragma once

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include "Util/NumType.h"
#include "Mapping/KeyFrame.h"

#include <sstream>
#include <fstream>

#define MAX_RES_PER_POINT 8

template<int ppp>
struct InputPointSparse
{
	float u;
	float v;
	float idpeth;
	float idepth_hessian;
	float relObsBaseline;
	int numGoodRes;
	unsigned char color[ppp];
	unsigned char status;
};

class KeyFrameDisplay
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        KeyFrameDisplay(KeyFrame keyFrame, float fx, float fy, float cx, float cy);
        ~KeyFrameDisplay();


        slam2::Matrix getPose();

        // copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
        bool refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity);

        // renders cam & pointcloud.
        void drawCam(float lineWidth = 1, float* color = 0, float sizeFactor=1);
        void drawPC(float pointSize);

        // render ground truth
        void drawGTCam(Sophus::Matrix4f m, float lineWidth, float* color, float sizeFactor);

        SE3 camToWorld;

        int id;
    	bool active;


    private:
        float fx,fy,cx,cy;
	    float fxi,fyi,cxi,cyi;
        int width, height;
        KeyFrame keyFrame;

        float my_scaledTH, my_absTH, my_scale;
        int my_sparsifyFactor;
        int my_displayMode;
        float my_minRelBS;
        bool needRefresh;

       	int numSparsePoints;
    	int numSparseBufferSize;
        InputPointSparse<MAX_RES_PER_POINT>* originalInputSparse;

        bool bufferValid;
        int numGLBufferPoints;
        int numGLBufferGoodPoints;
        pangolin::GlBuffer vertexBuffer;
        pangolin::GlBuffer colorBuffer;        
};